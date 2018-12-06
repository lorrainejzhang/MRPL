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
        offx; offy; offth;
    end

    methods (Static = true)
        function [x, y, th] = acquisitionPose(x, y, th, totalOffset)
%             totalOffset = robFrontOffset + objFaceOffset;% - moreOffset;
            totalOffset = totalOffset + 0.05;
            x = x - totalOffset * cos(th); 
            y = y - totalOffset * sin(th);
        end
    end
    
    methods
        function obj = mrplSystem(feedbackOn, mapOn)
            c = context();
            c.feedbackOn = feedbackOn;
            obj.follower = trajectoryFollower();
            obj.x1 = 0.75*0.3048; obj.y1 = 0.75*0.3048; obj.th1 = pi/2;
            eBot = simBot(c.robot, obj.x1, obj.y1, obj.th1);
            c.robot.encoders.NewMessageFcn=@eBot.listener;
            if mapOn
                c.robot.laser.NewMessageFcn = @eBot.laserListener;
            end
            obj.estBot = eBot;
            obj.context = c;
            % STUFF HERE
%             obj.x1 = 0; obj.y1 = 0; obj.th1 = 0;
            obj.offx = obj.x1; obj.offy = obj.y1; obj.offth = obj.th1;
            obj.i = 0;
            size = 10000;
            obj.xs = zeros(1,size); obj.ys = zeros(1,size); obj.ths = zeros(1,size);
            obj.enposxs = zeros(1,size); obj.enposys = zeros(1,size);
            obj.enposths = zeros(1,size);
            obj.ts = zeros(1,size);
        end
        
        function [x3, y3, th3] = absToRel(obj, x2, y2, th2)
%             x3u = x0 - obj.x1;
%             y3u = y0 - obj.y1;
%             x = x3u*cos(-obj.th1) - y3u*sin(-obj.th1);
%             y = x3u*sin(-obj.th1) + y3u*cos(-obj.th1);
%             th = th0 - obj.th1;
%             disp(th);
% %             obj.offx = obj.x1; obj.offy = obj.y1; obj.offth = obj.th1;
% %             obj.x1 = x0; obj.y1 = y0; obj.th1 = th0;
% %             x = x0 - obj.x1;
% %             y = y0 - obj.y1;
%             th = atan2(sin(th), cos(th));
            x3u = x2 - obj.x1;
            y3u = y2 - obj.y1;
            x3 = x3u*cos(-obj.th1) - y3u*sin(-obj.th1);
            y3 = x3u*sin(-obj.th1) + y3u*cos(-obj.th1);
            th3 = th2 - obj.th1;
            th3 = atan2(sin(th3),cos(th3));
            
        end
        
        function [x, y, th] = relToAbs(obj, x0, y0,th0)
            x = x0*cos(-obj.th1) + y0*sin(-obj.th1) + obj.x1;
            y = -x0*sin(-obj.th1) + y0*cos(-obj.th1) + obj.y1;
            th = th0 + obj.th1;
        end
        
        function executeTrajectory(obj, traj, ang)
            first = true;
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
%                 xi = x*cos(-obj.offth) + y*sin(-obj.offth) + obj.offx;
%                 yi = -x*sin(-obj.offth) + y*cos(-obj.offth) + obj.offy;
%                 thi = th + obj.offth;
%                 
                xi = x*cos(-obj.th1) + y*sin(-obj.th1) + obj.x1;
                yi = -x*sin(-obj.th1) + y*cos(-obj.th1) + obj.y1;
                thi = th + obj.th1;
                %fprintf("xi %f, yi %f, thi %f\n",xi,yi,thi);
                obj.i = obj.i + 1;
                obj.xs(obj.i) = xi;
                obj.ys(obj.i) = yi;
                obj.ths(obj.i) = thi;
                obj.enposxs(obj.i) = obj.estBot.enposx; obj.enposys(obj.i) = obj.estBot.enposy;
                obj.enposths(obj.i) = obj.estBot.enposth;

                obj.ts(obj.i) = T + obj.follower.startTime;

                obj.follower.sendVel(obj.context.robot, T, traj, xi, yi, thi, ...
                    obj.estBot.enposx,obj.estBot.enposy,obj.estBot.enposth, obj.estBot.goodT, obj.context.feedbackOn, ang);
                o = obj.estBot.startX; oo = obj.estBot.startY;
                plot(obj.xs(1:obj.i),obj.ys(1:obj.i),obj.enposxs(1:obj.i),obj.enposys(1:obj.i), ...
                     [0,0,obj.estBot.l] , [obj.estBot.l,0,0] ,'g', ... 
                     obj.estBot.xMap(1:obj.estBot.len)-o,obj.estBot.yMap(1:obj.estBot.len)-oo,'ro');
                xlim([-.05 4.05*.3048]);
                ylim([-.05 4.05*.3048]);
                pause(.05);
            end
            obj.context.robot.sendVelocity(0,0);            
        end
        
        function executeTrajectoryToRelativePose(obj, x3, y3, th3)
            %Have to call cubicSpiralTrajectory.makeLookupTable(100);
            sgn = 1;
            traj = cubicSpiralTrajectory.planTrajectory(x3, y3, th3, sgn);
            traj.planVelocities(obj.context.maxV);
%             obj.VARR = traj.VArray;
%             obj.DARR = traj.distArray;
            obj.executeTrajectory(traj, false);
            
            [x2, y2, th2] = obj.relToAbs(x3, y3, th3);
            obj.offx = obj.x1; obj.offy = obj.y1; obj.offth = obj.th1;
            obj.x1 = x2; obj.y1 = y2; obj.th1 = th2;
        end
        
        function executeTrajectoryToAbsPose(obj, x2, y2, th2)
            x3u = x2 - obj.x1;
            y3u = y2 - obj.y1;
            x3 = x3u*cos(-obj.th1) - y3u*sin(-obj.th1);
            y3 = x3u*sin(-obj.th1) + y3u*cos(-obj.th1);
            th3 = th2 - obj.th1;
            
            obj.offx = obj.x1; obj.offy = obj.y1; obj.offth = obj.th1;
            obj.x1 = x2; obj.y1 = y2; obj.th1 = th2;
            
            obj.executeTrajectoryToRelativePose(x3, y3, th3);
        end
        
        function executeTrap(obj, line, sf, sgn)
            if line
                t = trapReferenceControl(true, sf, sgn);
                obj.executeTrajectory(t, ~line);
                %obj.offx = obj.x1; obj.offy = obj.y1;
                obj.x1 = obj.x1 - sf * cos(obj.th1);
                obj.y1 = obj.y1 - sf * sin(obj.th1);
            else
                t = trapReferenceControl(false, sf, sgn);
                obj.executeTrajectory(t, ~line);
                %obj.offth = obj.th1;
                obj.th1 = obj.th1 + sf;
                obj.th1 = atan2(sin(obj.th1),cos(obj.th1));
                
            end
            %obj.executeTrajectory(t, ~line);    
        end
        
        function pickDropObject(obj, pick, x0, y0, th0)
            % absolute input pose
            % pick = true for pickup, pick = false for dropoff
            if pick
                obj.context.robot.forksDown();
            end
            [xo, yo, tho] = obj.absToRel(x0, y0, th0);
            %fprintf("x0 %f, y0 %f, tho %f",x0,y0,th0);
            if pick
                [x, y, th] = mrplSystem.acquisitionPose(xo, yo, tho, .15);
                obj.executeTrajectoryToRelativePose(x,y,th);
                pause(1);
                
            
                range = rangeImage(obj.context.robot);
                scatter(range.xArray, range.yArray)
                [xo, yo, tho] = range.findClosestSail();
                obj.executeTrajectoryToRelativePose(xo,yo,tho);
                
                pause(1);
            
            else
                disp([xo, yo, tho]); 
                obj.executeTrajectoryToRelativePose(xo,yo,tho);
                pause(1);
            end
            
            
            if pick 
                obj.context.robot.forksUp();
            else
                obj.context.robot.forksDown();
            end
            pause(1);
            obj.executeTrap(true, .15, -1);
            obj.executeTrap(false, pi, 1);
%             disp([obj.x1, obj.y1, obj.th1]);
%             disp([obj.offx, obj.offy, obj.offth]);
            pause(1);
        end
    end
end
