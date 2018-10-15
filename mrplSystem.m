classdef mrplSystem < handle
    properties
        context;
        follower;
        x1; y1; th1;
        estBot;
    end
    
%     methods (Static = true)        
%         function mrpl = executeTrajectoryToRelativePose(x, y, th, maxV, feedbackOn)
%            %Have to call cubicSpiralTrajectory.makeLookupTable(100);
%            traj = cubicSpiralTrajectory.planTrajectory(x, y, th, 1);
%            traj.planVelocities(maxV);
%            mrpl = mrplSystem;
%            mrpl.follower = trajectoryFollower();
%            mrpl.executeTrajectory(traj, feedbackOn);
%         end
%     end    
    
    methods
        function obj = mrplSystem()
            c = context();
            obj.follower = trajectoryFollower();
            eBot = simBot(c.robot);
            c.robot.encoders.NewMessageFcn=@eBot.listener;
            obj.estBot = eBot;
            obj.context = c;
            obj.x1 = 0; obj.y1 = 0; obj.th1 = 0;
            %obj.x1 = .3048; obj.y1 = .3048; obj.th1 =  pi/4 + pi/8;
            
        end
        
        function executeTrajectory(obj, traj, feedbackOn)
            %a = obj.estBot
            %obj.context.robot.encoders.NewMessageFcn=@obj.estBot.listener;
            size = 10000;
            %dur = traj.getTrajectoryDuration();
            xs = zeros(1,size);
            ys = zeros(1,size);
            enposxs = zeros(1,size);
            enposys = zeros(1,size);
            % vs = zeros(1,size);
            i = 0;
            first = true;
            while (i < 200)
                if (first)
                    obj.context.goodT = obj.estBot.goodT;
                    startTime = obj.context.getLocalTime();
                    % bump traj and traj final time by initial clock read 
                    %tFinal = tFinal + startTime; 
                    obj.follower.setStartTime(startTime); 
                    first = false;
                end
                T = obj.context.getLocalTime() - obj.follower.startTime;
                %disp(T)
                pose = traj.getPoseAtTime(T);
                x = pose(1); y = pose(2); th = pose(3);
                obj.follower.sendVel(obj.context.robot, T, traj, ...
                    obj.estBot.enposx,obj.estBot.enposy,obj.estBot.enposth, obj.estBot.goodT, feedbackOn);
                
                i = i + 1;
                %disp(i)
                xs(i) = x; ys(i) = y;
                enposxs(i) = obj.estBot.enposx; enposys(i) = obj.estBot.enposy;
                
                pause(.05);
            end
            obj.context.robot.sendVelocity(0,0);
            obj.context.robot.shutdown();
%             obj.xx = xs(1:i);
%             obj.yy = ys(1:i);
            plot(xs(1:i),ys(1:i),enposxs(1:i),enposys(1:i))
        end
        
        function executeTrajectoryToRelativePose(obj, x3, y3, th3)
            %Have to call cubicSpiralTrajectory.makeLookupTable(100);
            sgn = 1;
            traj = cubicSpiralTrajectory.planTrajectory(x3, y3, th3, sgn);
            traj.planVelocities(obj.context.maxV);
%             obj.VARR = traj.VArray;
%             obj.DARR = traj.distArray;
            obj.executeTrajectory(traj, obj.context.feedbackOn);
        end
        
        function executeTrajectoryToAbsPose(obj, x2, y2, th2)
            x3u = x2 - obj.x1;
            y3u = y2 - obj.y1;
            x3 = x3u*cos(-obj.th1) - y3u*sin(-obj.th1);
            y3 = x3u*sin(-obj.th1) + y3u*cos(-obj.th1);
            th3 = th2 - obj.th1;
            obj.executeTrajectoryToRelativePose(x3, y3, th3);
        end
            
   end
end
