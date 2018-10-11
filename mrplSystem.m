classdef mrplSystem < handle
    properties
        xx;
        yy;
    end
    
    methods (Static = true)        
        function thing = executeTrajectoryToRelativePose(x, y, th, maxV, feedbackOn)
           %Have to call cubicSpiralTrajectory.makeLookupTable(100);
           traj = cubicSpiralTrajectory.planTrajectory(x, y, th, 1);
           traj.planVelocities(maxV);
           mrpl = mrplSystem;
           mrpl.executeTrajectory(traj, feedbackOn);
           thing = mrpl;
        end
    end
    
    methods
        function executeTrajectory(obj, traj, feedbackOn)
            global enposx;
            global enposy;
            global enposth;
            global oldLeft;
            global oldRght;
            global oldt;
            global robot;
            global goodT;
            global init;
            init = 1;
            tread = 0.085;
            robot = raspbot();
            enposx = 0.0;
            enposy = 0.0;
            enposth = 0.0;
            oldLeft = 0.0;
            oldRght = 0.0;
            oldt = 0.0;
            goodT = 0;
            robot.encoders.NewMessageFcn=@encoderEventListener;
            
            follower = trajectoryFollower();

            size = 100000;
            dur = traj.getTrajectoryDuration();
            disp(dur);
            dt = dur/size;
            % ts = zeros(1,size);
            xs = zeros(1,size);
            ys = zeros(1,size);
            enposxs = zeros(1,size);
            enposys = zeros(1,size);
            % vs = zeros(1,size);
            tread = 0.085;
            i = 0;
            T = 0;
            Ti = tic();
            while(T <= dur + 1)
                T = toc(Ti);

                %ts(i) = (i-1)*dt;
                pose = traj.getPoseAtTime(T);
                x = pose(1);
                y = pose(2);
                th = pose(3);
                %disp(obj.enposx)
                follower.sendVel(robot, T, traj,enposx,enposy,enposth, goodT, feedbackOn);

%                 if abs(x) > 1
%                     x = 0;
%                 end
%                 if abs(y) > 1
%                     y = 0;
%                 end
                i = i + 1;
                xs(i) = x; ys(i) = y;
                
                enposxs(i) = enposx; enposys(i) = enposy;
                
                pause(.05);
            end
            robot.sendVelocity(0,0);
            robot.shutdown();
            obj.xx = xs(1:i);
            obj.yy = ys(1:i);
            plot(xs(1:i),ys(1:i),enposxs(1:i),enposys(1:i))
        end
        
        function encoderEventListener(obj, handle, event)
            %Change properties to be mrplSystem.prop?
            tread = 0.085;
            encoderDataTimestamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1000000000.0;
            dt = encoderDataTimestamp - obj.oldt;
            %disp(dt);
            %disp(goodT)
            obj.goodT = obj.goodT + dt;
            %disp(goodT)
            obj.oldt = encoderDataTimestamp;

            dleft = obj.robot.encoders.LatestMessage.Vector.X - obj.leftFirst - obj.oldLeft;
            drght = obj.robot.encoders.LatestMessage.Vector.Y - obj.rghtFirst - obj.oldRght;
            obj.oldLeft = obj.robot.encoders.LatestMessage.Vector.X - obj.leftFirst;
            obj.oldRght = obj.robot.encoders.LatestMessage.Vector.Y - obj.rghtFirst;
            envl = dleft/dt; 
            envr = drght/dt;
            env = (envl+envr)/2;
            enangVel = (envr-envl)/tread;
            ds = env*dt;
            endth = enangVel*dt/2;
            obj.enposth = obj.enposth + endth;
            endx = cos(obj.enposth)*ds;
            endy = sin(obj.enposth)*ds;
            obj.enposth = obj.enposth + endth;

            %disp(obj.enposx)
            obj.enposx = obj.enposx + endx;
            obj.enposy = obj.enposy + endy;
            %disp(obj.enposx)

            pause(.05);
        end
   end
end
