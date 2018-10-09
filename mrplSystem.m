classdef mrplSystem
    properties
        robot = raspbot();
        enposy = 0;
        enposth = 0;
        oldLeft = 0;
        oldRght = 0;
        oldt = 0;
        leftFirst = 0;
        rghtFirst = 0;
        goodT = 0;
        offset = 0;
    end
    
    methods        
        function executeTrajectoryToRelativePose(obj, x, y, th, maxV, feedbackOn)
           %Have to call cubicSpiralTrajectory.makeLookupTable(100);
           traj = cubicSpiralTrajectory.planTrajectory(x, y, th, 1);
           traj.planVelocities(maxV);
           executeTrajectory(traj, feedbackOn);
        end
        
        function executeTrajectory(obj, traj, feedbackOn)
            obj.robot.encoders.NewMessageFcn=@encoderEventListener;
            obj.leftFirst = obj.robot.encoders.LatestMessage.Vector.X;
            obj.rghtFirst = obj.robot.encoders.LatestMessage.Vector.Y;
            
            follower = trajectoryFollower();

            size = 100000;
            dur = traj.getTrajectoryDuration;
            dt = dur/size;
            % ts = zeros(1,size);
            xs = zeros(1,size);
            ys = zeros(1,size);
            obj.enposxs = zeros(1,size);
            obj.enposys = zeros(1,size);
            % vs = zeros(1,size);
            tread = 0.085;
            i = 1;
            T = 0;
            Ti = tic();
            while(T <= dur + 1)
                T = toc(Ti);

                %ts(i) = (i-1)*dt;
                [x, y, th] = traj.getPoseAtTime(T);
                %disp(obj.enposx)
                follower.sendVel(obj.robot, T, traj,
                                 obj.enposx,obj.enposy,obj.enposth, obj.goodT, feedbackOn);

                if abs(x) > 1
                    x = 0;
                end
                if abs(y) > 1
                    y = 0;
                end
                xs(i) = x; ys(i) = y;
                obj.enposxs(i) = obj.enposx; obj.enposys(i) = obj.enposy;
                i = i + 1;
                pause(.05);
            end
            obj.robot.sendVelocity(0,0);
            obj.robot.shutdown();
            %plot(xs,ys)
            plot(xs,ys,obj.enposxs,obj.enposys)
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
