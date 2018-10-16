classdef controller < handle
    properties
        oldBackV = 0;
        oldBackW = 0;
        VInt = 0;
        WInt = 0;
        oldT = 0;
        lastPose;
        started = false;
    end
    methods
        function [backVTot, backWTot] = feedback(obj, x, y, th, enposx,enposy,enposth, V, T)
            errx = (x - enposx);
            erry = (y - enposy);
            errth = (th - enposth);
            %errPoseWorld = pose(errx,erry,errth);
            %disp(errPoseWorld.getPoseVec)
            %transform = errPoseWorld.aToBRot();
            mat = zeros(2,2);
            th = enposth;

            mat(1,1) =  cos(th); mat(1,2) = -sin(th);
            mat(2,1) =  sin(th); mat(2,2) =  cos(th);
            %disp(mat)
         
            rrp = (mat^-1)*([errx;erry]);
            
            %disp(rrp)
            tau = 2;
            kx = 1/(tau);
            if abs(V) < .1
                ky = 0;
            else
                ky = 2/((tau^2)*abs(V));
            end
            kth = kx;
%             kx = 0;
%             ky = 0;
%             kth = 0;
            backV = kx*rrp(1);
            backW = ky*rrp(2) + kth * atan2(sin(errth),cos(errth));
            %fprintf("errx: %f, rrp1: %f, erry: %f, rrp2: %f",errx,rrp(1),erry,rrp(2));
            delTime = T - obj.oldT;
            obj.oldT = T;
            backVDer = (backV - obj.oldBackV) / delTime;
            obj.oldBackV = backV;
            backWDer = (backW - obj.oldBackW) / delTime;
            obj.oldBackW = backW;
            obj.VInt = obj.VInt + backV * delTime;
            obj.WInt = obj.WInt + backW * delTime;
            eiVMax = .2;
            eiWMax = .2;
            if (abs(obj.VInt) > eiVMax)
                obj.VInt = sign(obj.VInt) * eiVMax;
            end
            if (abs(obj.WInt) > eiWMax)
                obj.WInt = sign(obj.WInt) * eiWMax;
            end
            kp = 1; % tau 4, kp 1, kd .2, ki .3
            kd = 0; % tau 3, 1, .2 .3  
            ki = 0;
            backVTot = kp*backV + kd*backVDer + ki*obj.VInt;
            backWTot = kp*backW + kd*backWDer + ki*obj.WInt;
        end    
        
        function initialize(obj, startPose)
            obj.lastPose = startPose;
            obj.started = false;
        end
    end
end
