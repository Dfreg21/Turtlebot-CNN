clear all
close all hidden
clc
%%
rosshutdown
rosinit("http://192.168.50.101:11311")

velPub = rospublisher("/bot1/cmd_vel","geometry_msgs/Twist") ;
velMsg = rosmessage(velPub);
camSub = rossubscriber("/bot1/raspicam_node/image/compressed","sensor_msgs/CompressedImage");
odomSub = rossubscriber("/bot1/odom","nav_msgs/Odometry");
resetPub = rospublisher("/bot1/reset","std_msgs/Empty");
resetMsg = rosmessage(resetPub);

send(resetPub,resetMsg)
pause(0.5)

R = .03;
L = .15;

kp_v = 0.3;
kp_w = 0.6;

p_plot = [];
pp_plot = [];
pd_plot = [];
qp_plot = [];
t_plot = [];

load cnn_net_digits
s = 120;

tic
%%
bandera = 1;
while toc<=120
    t = toc;
    %%
    if bandera == 1
        camMsg = receive(camSub);
        camMsg.Format = 'bgr8; jpeg compressed bgr8';

        img = readImage(camMsg);
        imshow(img)
        I = rgb2gray(img);

        BW = imbinarize(I,'global');
        BW = imcomplement(BW);

        img = imresize(BW,[28 28]);
        YPred = classify(net,255*img);
        disp(YPred)
        bandera = 0;
    end

    %%
    Pr=double(YPred);
    
    if Pr == 0
        velMsg.Linear.X = 0.0;
        velMsg.Angular.Z = 0.0;
        send(velPub,velMsg);
        
        send(resetPub,resetMsg);
        pause(0.5)
        S=0;
    elseif Pr >= 6
        velMsg.Linear.X = 0.0;
        velMsg.Angular.Z = 0.0;
        send(velPub,velMsg);
        
        send(resetPub,resetMsg);
        pause(0.5)
        S=0;
    else
        S = Pr;
    end
    imshowpair(I,BW,'montage')

    %%
    % Odometria
    odomMsg = receive(odomSub);
    pose = odomMsg.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;

    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = angles(1);
    
    [xd,yd] = Trajectory(t,S);
            
    e_v = sqrt((xd-x)^2+(yd-y)^2);

    e_w = atan2(yd-y,xd-x) - theta;
    e_w = atan2(sin(e_w),cos(e_w));

    v = kp_v*e_v;
    w = kp_w*e_w;

    v = max(v,-0.2);
    v = min(v,0.2);

    w = max(w,-0.5);
    w = min(w,0.5);
    
    velMsg.Linear.X = v;
    velMsg.Angular.Z = w;

    send(velPub,velMsg);
    
    wr = (2*v+w*L)/(2*R);
    wl = (2*v-w*L)/(2*R);

    p_plot = [p_plot [x y]'];
    % pp_plot = [pp_plot pp];
    pd_plot = [pd_plot [xd; yd]];
    qp_plot = [qp_plot [wr; wl]];
    t_plot = [t_plot t];
    
%     cla
%     hold on
%     imshow(img)
%     drawnow
end

%%
velMsg.Linear.X = 0.0;
velMsg.Angular.Z = 0.0;
send(velPub,velMsg)

send(resetPub,resetMsg)
pause(0.5)

%%
if ~isempty(p_plot)
    figure
    hold on
    grid on
    plot(pd_plot(1,:),pd_plot(2,:),'r','LineWidth',2)
    plot(p_plot(1,:),p_plot(2,:),'b','LineWidth',2)
    title('Trayectoria')
    xlabel('x')
    ylabel('y')
    legend('referencia','actual','Location','best')

    figure
    hold on
    grid on
    plot(t_plot,p_plot(1,:),'m','LineWidth',2)
    plot(t_plot,p_plot(2,:),'g','LineWidth',2)
    plot(t_plot,p_plot(3,:),'k','LineWidth',2) 
    title('p')
    xlabel('t')
    ylabel('m, rad')
    legend('x','y','\theta','Location','best')
    
    figure
    hold on
    grid on
    plot(t_plot,pp_plot(1,:),'m','LineWidth',2)
    plot(t_plot,pp_plot(2,:),'g','LineWidth',2)
    plot(t_plot,pp_plot(3,:),'k','LineWidth',2) 
    title('p_p')
    xlabel('t')
    ylabel('m/s, rad/s')
    legend('x_p','y_p','\theta_p','Location','best')
    
    figure
    hold on
    grid on
    plot(t_plot,qp_plot(1,:),'m','LineWidth',2)
    plot(t_plot,qp_plot(2,:),'g','LineWidth',2)
    title('q_p')
    xlabel('t')
    ylabel('m/s, rad/s')
    legend('wr','wl','Location','best')
end
%%
function [x,y] = Trajectory (t,trac)
    switch trac
        case 0 
            A = 0;
            T = 0;
            
            x = 0;
            y = 0;
        case 1
            A = 0.5;
            T = 2.0;
            
            x = 0.1*t;
            y = A*sin(2*pi*(1/T)*x);
        case 2
            A = 0.8;
            T = 2.0;
            
            x = 0.1*t;
            y = A*sin(2*pi*(1/T)*x);
            
            if y>0.5
                y = 0.5;
            end
            
            if y<-0.5
                y = -0.5;
            end
        case 3
            dx = 0.5;
            dy = 0.5;
            r = 1.0;
            
            x = dx + r*cos(0.1*t);
            y = dy + r*sin(0.1*t);
        case 4
            dx = 0.5;
            dy = 0.5;
            r = 0.8 + 0.2*cos(3*0.1*t);
            
            x = dx + r*cos(0.1*t);
            y = dy + r*sin(0.1*t);
        case 5
            z = 0.1*t;
            
            if 0<z && z<=2.5
                x = z;
                y = 0;
            elseif 2.5<z && z<=5
                x = 2.5;
                y = z-2.5;
            elseif 5<z && z<=10
                x = 7.5-z;
                y = 2.5;
            elseif 10<z && z<=15
                x = -2.5;
                y = 12.5-z;
            elseif 15<z && z<=20
                x = z-17.5;
                y = -2.5;
            elseif 20<z && z<=22.5
                x = 2.5;
                y = z-22.5;
            elseif 22.5<z && z<=25
                x = 25.-z;
                y = 0;
            else
                x = 0;
                y = 0;
            end
    end
end