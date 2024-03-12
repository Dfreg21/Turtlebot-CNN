clear all
close all hidden
clc

R = .0975;
L = .381/2;

S = 1;

kp_v = 0.2;
kp_w = 1.5;

p_plot = [];
pp_plot = [];
pd_plot = [];
qp_plot = [];
t_plot = [];

VREP = remApi('remoteApi'); % Crear el objeto vrep y cargar la libreria
ClientID = VREP.simxStart('127.0.0.1',19999,true,true,5000,5); % Iniciar conexión con el simulador

if ClientID~=-1
    [~,Pioneer] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx',VREP.simx_opmode_oneshot_wait);
    [~,motorL] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_leftMotor',VREP.simx_opmode_oneshot_wait);
    [~,motorR] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_rightMotor',VREP.simx_opmode_oneshot_wait);
 
    tic 

    while VREP.simxGetConnectionId(ClientID)~=-1
        [aux_position,position] = VREP.simxGetObjectPosition(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [aux_orientation,orientation] = VREP.simxGetObjectOrientation(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [aux_vel,linearVelocity,angularVelocity] = VREP.simxGetObjectVelocity(ClientID,Pioneer,VREP.simx_opmode_streaming);
        
        if aux_position==0 && aux_orientation==0 && aux_vel==0
            p = [position(1) position(2) orientation(3)]';
            pp = [linearVelocity(1) linearVelocity(2) angularVelocity(3)]';
            
            t = toc;
            
            [xd,yd] = Trajectory(t,S);
            
            e_v = sqrt((xd-p(1))^2+(yd-p(2))^2);
            
            e_w = atan2(yd-p(2),xd-p(1)) - p(3);
            e_w = atan2(sin(e_w),cos(e_w));
            
            v = kp_v*e_v;
            w = kp_w*e_w;
            
            wr = (2*v+w*L)/(2*R);
            wl = (2*v-w*L)/(2*R);
            
            VREP.simxSetJointTargetVelocity(ClientID,motorL,wl,VREP.simx_opmode_streaming);
            VREP.simxSetJointTargetVelocity(ClientID,motorR,wr,VREP.simx_opmode_streaming);

            p_plot = [p_plot p];
            pp_plot = [pp_plot pp];
            pd_plot = [pd_plot [xd; yd]];
            qp_plot = [qp_plot [wr; wl]];
            t_plot = [t_plot t];
        end
    end
end

VREP.simxFinish(ClientID); %Terminar la conexion

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
