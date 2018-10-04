classdef trajectoryFollower
    methods
        function [] = sendVel(obj, robot, T, traj, enposx,enposy,enposth, goodT, ref)
            tread = 0.085;
            V = traj.getVelAtTime(T);
            w = traj.getOmegaAtTime(T);
            %disp(enposx)
            cont = controller();
            %backV = 0; backW = 0;
            [backV, backW] = cont.feedback(T, traj, enposx,enposy,enposth, V);
%             disp(backV)
%             disp(backW)
            vr = (V + backV) + (tread / 2) * (w + backW);
            vl = (V + backV) - (tread / 2) * (w + backW);
            
            if isnan(vr) || isinf(vr)
                %disp('gs')
                vr = 0;
            end
            if isnan(vl) || isinf(vl)
                %disp('sds')
                vl = 0;
            end
            if (T > ref.getTrajectoryDuration) && (T < ref.getTrajectoryDuration + 2)
                vl = 0;
                vr = 0;
            end
%             disp(vl)
%             disp(vr)

            robot.sendVelocity(vl,vr);
        end
    end
end