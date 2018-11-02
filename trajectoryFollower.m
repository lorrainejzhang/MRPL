classdef trajectoryFollower < handle
    properties
        bv = 0;
        bw = 0;
        trajectory;
        cont = controller();
        startPose;
        startTime;
    end
    methods
%         function obj = trajectoryFollower(traj)
%             obj.trajectory = traj;
%             obj.cont = controller();  
%         end
        
        function [] = sendVel(obj, robot, T, traj, x, y, th, enposx,enposy,enposth, goodT, feedbackOn, ang)
            V = traj.getVAtTime(T);
            w = traj.getwAtTime(T);

            [backV, backW] = obj.cont.feedback(x, y, th, enposx,enposy,enposth, V, T);
            if ang
                %fprintf("V %f, w %f, backv %f, backw %f\n",V,w,backV,backW);
                backV = 0; % backV is too high when rotating from the trapezoidal, idk why
            end
            backV = feedbackOn * backV;
            backW = feedbackOn * backW;
            obj.bv = backV;
            obj.bw = backW;
            vr = (V + backV) + (robotModel.tread / 2) * (w + backW);
            vl = (V + backV) - (robotModel.tread / 2) * (w + backW);
            
            if isnan(vr) || isinf(vr)
                vr = 0;
            end
            if isnan(vl) || isinf(vl)
                vl = 0;
            end
            dur = traj.getTrajectoryDuration;
            if (T > dur)
                vl = 0;
                vr = 0;
            end
            %disp(vl)
            robot.sendVelocity(vl,vr);
        end
        
        function loadTrajectory(obj, trajectory, startPose)
            obj.trajectory = trajectory;
            obj.startPose = startPose;
            obj.cont.initialize(startPose);
        end
        
        function setStartTime(obj, startTime)
            obj.startTime = startTime;
        end
    end
end
