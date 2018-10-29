classdef mrplSystem < handle
    properties
        context;
        follower;
        x1; y1; th1;
        estBot;
        list;
        i;
        xs; ys; ths; ts;
        enposxs; enposys; enposths;
        oldth; offx; offy; offth;
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

    methods (Static = true)
        function [x, y, th] = acquisitionPose(x, y, th, robFrontOffset,objFaceOffset,moreOffset)
            totalOffset = robFrontOffset + objFaceOffset;% - moreOffset;
            totalOffset = totalOffset + 0.07;
            x = x - totalOffset * cos(th); 
            y = y - totalOffset * sin(th);
            th = th;
        end
    end
    
    methods
        function obj = mrplSystem(feedbackOn)
            c = context();
            c.feedbackOn = feedbackOn;
            obj.follower = trajectoryFollower();
            eBot = simBot(c.robot);
            c.robot.encoders.NewMessageFcn=@eBot.listener;
            obj.estBot = eBot;
            obj.context = c;
            obj.x1 = 0; obj.y1 = 0; obj.th1 = 0;
            obj.i = 0;
            size = 10000;
            obj.xs = zeros(1,size); obj.ys = zeros(1,size); obj.ths = zeros(1,size);
            obj.enposxs = zeros(1,size); obj.enposys = zeros(1,size);
            obj.enposths = zeros(1,size);
            obj.ts = zeros(1,size);
            
            obj.offx = 0; obj.offy = 0; obj.offth = 0; obj.oldth = 0;
            
            %obj.x1 = .3048; obj.y1 = .3048; obj.th1 =  pi/4 + pi/8;
            
        end
        
        function executeTrajectory(obj, traj)
            %a = obj.estBot
            %obj.context.robot.encoders.NewMessageFcn=@obj.estBot.listener;
            %dur = traj.getTrajectoryDuration();
           
%             enposxs = zeros(1,size);
%             enposys = zeros(1,size);
            % vs = zeros(1,size);
            %i = 0;
            first = true;
            %disp(obj.follower.startTime
            dur = traj.getTrajectoryDuration;
            T = 0;
            while (T < dur)
                if (first)
                    %obj.context.goodT = obj.estBot.goodT;
                    startTime = obj.context.getLocalTime();
                    % bump traj and traj final time by initial clock read 
                    %tFinal = tFinal + startTime; 
                    obj.follower.setStartTime(startTime); 
                    first = false;
                end
                T = obj.context.getLocalTime() - obj.follower.startTime;
                %T = obj.estBot.goodT - obj.follower.startTime;
                %disp(T)
                pose = traj.getPoseAtTime(T);
                x = pose(1); y = pose(2); th = pose(3);
                %disp("here");
                %disp(obj.oldth);
                %disp(obj.offx);
                xi = x*cos(-obj.oldth) + y*sin(-obj.oldth) + obj.offx;
                yi = -x*sin(-obj.oldth) + y*cos(-obj.oldth) + obj.offy;
                thi = th + obj.offth;

                obj.i = obj.i + 1;
                %disp(i)
                obj.xs(obj.i) = xi;
                obj.ys(obj.i) = yi;
                obj.ths(obj.i) = thi;
                obj.enposxs(obj.i) = obj.estBot.enposx; obj.enposys(obj.i) = obj.estBot.enposy;
                obj.enposths(obj.i) = obj.estBot.enposth;

                obj.ts(obj.i) = T + obj.follower.startTime;

                obj.follower.sendVel(obj.context.robot, T, traj, xi, yi, thi, ...
                    obj.estBot.enposx,obj.estBot.enposy,obj.estBot.enposth, obj.estBot.goodT, obj.context.feedbackOn);

                
                pause(.05);
            end
            obj.context.robot.sendVelocity(0,0);
            %disp("----------------")
            %obj.context.robot.shutdown();
%             obj.xx = xs(1:i);
%             obj.yy = ys(1:i);
            
        end
        
        function executeTrajectoryToRelativePose(obj, x3, y3, th3)
            %Have to call cubicSpiralTrajectory.makeLookupTable(100);
            sgn = 1;
            traj = cubicSpiralTrajectory.planTrajectory(x3, y3, th3, sgn);
            traj.planVelocities(obj.context.maxV);
%             obj.VARR = traj.VArray;
%             obj.DARR = traj.distArray;
            obj.executeTrajectory(traj);
        end
        
        function executeTrajectoryToAbsPose(obj, x2, y2, th2)
            x3u = x2 - obj.x1;
            y3u = y2 - obj.y1;
            x3 = x3u*cos(-obj.th1) - y3u*sin(-obj.th1);
            y3 = x3u*sin(-obj.th1) + y3u*cos(-obj.th1);
            th3 = th2 - obj.th1;
            obj.oldth = obj.th1;
            %obj.context.robot.vel_pub.NumSubscribers = 1;
            %disp(obj.context.robot.vel_pub)
            %obj.offx = obj.x1; obj.offy = obj.y1;
            obj.offx = obj.x1; obj.offy = obj.y1; obj.offth = obj.th1;
            obj.x1 = x2; obj.y1 = y2; obj.th1 = th2;
            
            obj.executeTrajectoryToRelativePose(x3, y3, th3);
        end
            
   end
end
