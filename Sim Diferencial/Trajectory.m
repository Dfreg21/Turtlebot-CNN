function [x,y] = Trajectory (t,trac)
    switch trac
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
          