classdef simBot < handle
    properties
        enposx; enposy; enposth; enpos;
        oldLeft; oldRght; oldt;
        robot;
        goodT;
        leftFirst; rghtFirst;
        lines_p1; lines_p2; l;
        local;
        gain;
        startX; startY; startTh;
        first;
        xMap; yMap; len;
    end
    methods
        function obj = simBot(robot)
            obj.robot = robot;
            obj.leftFirst = robot.encoders.LatestMessage.Vector.X;
            obj.rghtFirst = robot.encoders.LatestMessage.Vector.Y;
            obj.enposx = 0.0;
            obj.enposy = 0.0;
            obj.enposth = 0.0;
            obj.enpos = [0.0,0.0,0.0];
            obj.oldLeft = 0.0;
            obj.oldRght = 0.0;
            obj.oldt = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
            obj.goodT = 0;
            l = 4*.3048;
            obj.l = l;
            lines_p1 = [[0;l] [0;0]];
            lines_p2 = [[0;0] [l;0]];
            obj.local = lineMapLocalizer(lines_p1,lines_p2,0.3,0.01,0.0005);
            obj.lines_p1 = lines_p1;
            obj.lines_p2 = lines_p2;
            obj.gain = .1;
            obj.first = true;
            obj.xMap = zeros(1,1000); obj.yMap = zeros(1,1000); obj.len = 1;
        end
        
        function [] = listener(obj, handle, event)
            encoderDataTimestamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1000000000.0;
            dt = encoderDataTimestamp - obj.oldt;
            obj.goodT = obj.goodT + dt;
            obj.oldt = encoderDataTimestamp;

%             vecx = obj.robot.encoders.LatestMessage.Vector.X;
%             vecy = obj.robot.encoders.LatestMessage.Vector.Y;
            vecx = event.Vector.X;
            vecy = event.Vector.Y;
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
            
            obj.enpos = [obj.enposx, obj.enposy, obj.enposth];
        end
        
        function [] = laserListener(obj, handle, event)
           % disp(event)
            rangeImagePolar = event.Ranges;
            xArr = zeros(1,360);
            yArr = zeros(1,360);
            thArr = zeros(1,360);

            j = 0;
            for i = 1:360
                r = rangeImagePolar(i);
                [x, y, th] = rangeImage.irToXy(i, r);
                if ~(r < .06 || r > 5*.3048)
                    j = j + 1;
                    xArr(j) = x;
                    yArr(j) = y;
                    thArr(j) = th;

                end
            end

            gx = xArr(1:15:j);
            gy = yArr(1:15:j);

            [success, outPose] = obj.local.refinePose(obj.enpos,[gx; gy],50);
            if success == 1
                %driver.drive(sys.context.robot, 1);

                x = outPose(1); y = outPose(2); th = outPose(3);
                if obj.first
                    obj.startX = x; obj.startY = y; obj.startTh = th;
                    obj.first = false;
                end
                %xArrWorld = zeros(1,length(gx));
                %yArrWorld = zeros(1,length(gx));
                for i = 1:length(gx)
                    xTemp = gx(i);
                    yTemp = gy(i);
                    a = xTemp * cos(-th) + yTemp * sin(-th) +x;
                    b  = -xTemp * sin(-th) + yTemp * cos(-th) +y;
                    %xArrWorld(i) =  a;
                    %yArrWorld(i) = b;
                    obj.xMap(i) = a;
                    obj.yMap(i) = b;
                end
                %obj.xMap = xArrWorld; obj.yMap = yArrWorld; 
                obj.len = length(gx);
                dx = obj.gain * ((x - obj.startX) - obj.enposx);
                dy = obj.gain * ((y - obj.startY) - obj.enposy);
                obj.enposx = obj.enposx + obj.gain * ((x - obj.startX) - obj.enposx);
                obj.enposy = obj.enposy + obj.gain * ((y - obj.startY) - obj.enposy);
                obj.enposth = obj.enposth + obj.gain * ((th - obj.startTh) - obj.enposth);
                obj.enposth = atan2(sin(obj.enposth), cos(obj.enposth));
                obj.enpos = [obj.enposx, obj.enposy, obj.enposth];
    %                 pause(.2);
            end


            
        end
    end
end
