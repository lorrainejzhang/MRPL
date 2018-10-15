classdef simBot < handle
    properties
        enposx;
        enposy;
        enposth;
        oldLeft;
        oldRght;
        oldt;
        robot;
        goodT;
        leftFirst;
        rghtFirst;
    end
    methods
        function obj = simBot(robot)
            obj.robot = robot;
            obj.leftFirst = robot.encoders.LatestMessage.Vector.X;
            obj.rghtFirst = robot.encoders.LatestMessage.Vector.Y;
            obj.enposx = 0.0;
            obj.enposy = 0.0;
            obj.enposth = 0.0;
            obj.oldLeft = 0.0;
            obj.oldRght = 0.0;
            obj.oldt = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
            obj.goodT = 0;
        end
        
        function [] = listener(obj, handle, event)
            encoderDataTimestamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1000000000.0;
            dt = encoderDataTimestamp - obj.oldt;
            obj.goodT = obj.goodT + dt;
            obj.oldt = encoderDataTimestamp;

            vecx = obj.robot.encoders.LatestMessage.Vector.X;
            vecy = obj.robot.encoders.LatestMessage.Vector.Y;
            dleft = vecx - obj.leftFirst - obj.oldLeft;
            drght = vecy - obj.rghtFirst - obj.oldRght;
            obj.oldLeft = vecx - obj.leftFirst;
            obj.oldRght = vecy - obj.rghtFirst;
            envl = dleft/dt; 
            envr = drght/dt;
            env = (envl+envr)/2;
            enangVel = (envr-envl)/robotModel.tread;
            ds = env*dt;
            endth = enangVel*dt/2;
            obj.enposth = obj.enposth + endth;
            endx = cos(obj.enposth)*ds;
            endy = sin(obj.enposth)*ds;
            obj.enposth = obj.enposth + endth;

            obj.enposx = obj.enposx + endx;
            obj.enposy = obj.enposy + endy;
        end
    end
end