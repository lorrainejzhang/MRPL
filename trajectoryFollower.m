classdef trajectoryFollower
    methods
        function [] = sendVel(obj, robot, T, traj, enposx,enposy,enposth, goodT, feedbackOn)
            tread = 0.085;
            V = traj.getVAtTime(T);
            w = traj.getwAtTime(T);
            %disp(enposx)
            cont = controller();
            %backV = 0; backW = 0;
            [backV, backW] = cont.feedback(T, traj, enposx,enposy,enposth, V);
%             disp(backV)
%             disp(backW)
            backV = feedbackOn * backV;
            backW = feedbackOn * backW;
            vr = (V + backV) + (tread / 2) * (w + backW);
            vl = (V + backV) - (tread / 2) * (w + backW);
            
            if isnan(vr) || isinf(vr)
                vr = 0;
            end
            if isnan(vl) || isinf(vl)
                vl = 0;
            end
            if (T > traj.getTrajectoryDuration) && (T < traj.getTrajectoryDuration + 2)
                vl = 0;
                vr = 0;
            end
            %disp(vl)
            %disp(vr)

            robot.sendVelocity(vl,vr);
        end
    end
end
