classdef robotTrajectory
    properties(Constant)
        meth = "spline"
    end
    properties
        t,v,x,y,th,s,w;
    end
    
    methods
        function obj = robotTrajectory(numSamples, refControl)
            dt = refControl.getTrajectoryDuration/numSamples;
            t = zeros(1,numSamples);
            v = zeros(1,numSamples);
            w = zeros(1,numSamples);
            x = zeros(1,numSamples);
            y = zeros(1,numSamples);
            th = zeros(1,numSamples);
            s = zeros(1,numSamples);
%             for i = 2 : numSamples
%                 t(i) = (i - 1)*dt;
%                 [V, ww] = refControl.computeControl(t(i));
%                 v(i) = V;
%                 w(i) = ww;
%                 disp(ww)
%                 ds = v(i)*dt; %distance travelled in this dt interval
%                 s(i) = s(i-1) + ds;
%                 th(i) = th(i-1) + w(i)*dt; %theta_i+1 = theta_i + w*dt
%                 %disp(th(i))
%                 x(i) = x(i-1) + cos(th(i))*ds; %x_i+1 = x_i + cos(theta_i+1)*ds
%                 y(i) = y(i-1) + sin(th(i))*ds; %y_i+1 = y_i + sin(theta_i+1)*ds
            for i = 1 : numSamples-1
                t(i) = (i - 1)*dt;
                [V, ww] = refControl.computeControl(t(i));
                v(i) = V;
                w(i) = ww;
                %disp(ww)
                ds = v(i)*dt; %distance travelled in this dt interval
                s(i+1) = s(i) + ds;
                th(i+1) = th(i) + w(i)*dt; %theta_i+1 = theta_i + w*dt
                %disp(th(i))
                x(i+1) = x(i) + cos(th(i+1))*ds; %x_i+1 = x_i + cos(theta_i+1)*ds
                y(i+1) = y(i) + sin(th(i+1))*ds; %y_i+1 = y_i + sin(theta_i+1)*ds
            end
            t(numSamples) = refControl.getTrajectoryDuration;
            v(numSamples) = 0;
            w(numSamples) = 0;

            obj.t = t; obj.v = v; obj.x = x; obj.y = y; obj.th = th; obj.s = s; obj.w = w;
           
        end
            
        function pose = getPoseAtTime(obj, t)
            x = interp1(obj.t, obj.x, t, obj.meth);
            y = interp1(obj.t, obj.y, t, obj.meth);
            th = interp1(obj.t, obj.th, t, obj.meth);
            pose = [x; y; th];
        end
        
        function vel = getVAtTime(obj, t)
            vel = interp1(obj.t, obj.v, t, obj.meth);
        end
        
        function omega = getwAtTime(obj, t)
            omega = interp1(obj.t, obj.w, t, obj.meth);
        end
        
        function dist = getDistAtTime(obj, t)
            dist = interp1(obj.t, obj.s, t, obj.meth);
        end    
    end
end