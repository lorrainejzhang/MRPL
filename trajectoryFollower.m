classdef trajectoryFollower < handle
    properties
        bv = 0;
        bw = 0;
    end
    methods
        function [] = sendVel(obj, robot, T, traj, enposx,enposy,enposth, goodT, feedbackOn)%, ref)
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
            obj.bv = backV;
            obj.bw = backW;
            vr = (V + backV) + (tread / 2) * (w + backW);
            vl = (V + backV) - (tread / 2) * (w + backW);
            
            if isnan(vr) || isinf(vr)
                vr = 0;
            end
            if isnan(vl) || isinf(vl)
                vl = 0;
            end
            dur = traj.getTrajectoryDuration;
            %dur = ref.getTrajectoryDuration;
            if (T > dur)% && (T < dur + 2)
                vl = 0;
                vr = 0;
            end
            %disp(vl)
            %disp(vr)

            robot.sendVelocity(vl,vr);
        end
    end
end
