classdef controller
    methods
        function [backV, backW] = feedback(obj, T, traj, enposx,enposy,enposth, V)
            [x, y, th] = traj.getPoseAtTime(T);
            %disp(enPose)
             %disp(x)
%             disp(enposx)
            errx = x - enposx;
            erry = y - enposy;
            errth = th - enposth;
            %errPoseWorld = pose(errx,erry,errth);
            %disp(errPoseWorld.getPoseVec)
            %transform = errPoseWorld.aToBRot();
            mat = zeros(2,2);
            th = enposth;

            mat(1,1) =  cos(th); mat(1,2) = -sin(th);
            mat(2,1) =  sin(th); mat(2,2) =  cos(th);
            %disp(mat)
         
            rrp = (mat^-1)*([errx;erry]);
            disp(rrp)
            kx = 1/(2*pi);
            ky = 2/((4*pi*pi)*abs(V));
            kth = kx;
            backV = kx*rrp(1);
            backW = ky*rrp(2) + kth * atan2(sin(errth),cos(errth));
        end        
    end
end