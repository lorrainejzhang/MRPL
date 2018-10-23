classdef rangeImage < handle
    properties(Constant)
        % thOffset = atan2(0.024, 0.28); from ex 1?
        
        maxUsefulRange = 1%2.0; %m
        minUsefulRange = 0.1; %m
        maxRangeForTarget = 10; %m
        
        sailWidth = 3.8/100; %m
        minNumPixInSail = 3; % Can change
        boundingBoxDiagError = .2; % Can change, too big!
        maxLambda1 = 1.3; % Can change, not sure what this is?
    end
    
    properties(Access = public)
        rArray = zeros(1, 360);
        thArray = zeros(1, 360);
        xArray = zeros(1, 360);
        yArray = zeros(1, 360);
        
        %These are in the [minUsefulRange, maxRangeForTarget] range
        %May not be 360 of them! That's why we've got numGoodPix
        goodRArray = [];
        goodThArray = [];
        goodXArray = [];
        goodYArray = [];
        numGoodPix = 0;
    end
    
    methods (Static = true)
        function [x, y, th] = irToXy(i, r) 
            th = (-5*pi/180) + (i - 1) * (pi/180); %- obj.thOffset; from ex 1?
            x = r * cos(th);
            y = r * sin(th);
        end
    end
    
    methods
        
        function obj = rangeImage(robot)
            [obj.xArray, obj.yArray, obj.thArray] = lab8(robot);
            for i = 1:360
%                 r = ranges(i);
                x = obj.xArray(i);
                y = obj.yArray(i);
                r = sqrt(x^2 + y^2);
                th = obj.thArray(i);
%                 [x y th] = irToXy(i, r);
%                 d = sqrt(x^2 + y^2);
%                 if (d < .1 || d > .95)
%                     x = 0; y = 0;
%                 end
%                 obj.rArray(i) = r;
%                 obj.thArray(i) = th;
%                 obj.xArray(i) = x;
%                 obj.yArray(i) = y;
                                
                if ~(r > obj.maxRangeForTarget || r < obj.minUsefulRange)

                    obj.numGoodPix = obj.numGoodPix + 1;
                    obj.goodRArray = [obj.goodRArray, r];
                    obj.goodThArray = [obj.goodThArray, th];
                    obj.goodXArray = [obj.goodXArray, x];
                    obj.goodYArray = [obj.goodYArray, y];
                end
            end 
            %obj.goodXArray = obj.xArray > obj.minUsefulRange && obj.xArray < obj.maxRangeForTarget;

        end
                
        function [sailCenterX, sailCenterY, sailTh] = findClosestSail(obj)
            closestSailDist = rangeImage.maxRangeForTarget + 1;
            sailCenterX = 0; sailCenterY = 0; sailTh = 0;
            for i = 1:obj.numGoodPix
                xi = obj.goodXArray(i);
                yi = obj.goodYArray(i);
                numPixNearby = 0; % Pix within half a sailWidth
                x = zeros(1,360);
                y = zeros(1,360);
                for j = 1:obj.numGoodPix
                    xj = obj.goodXArray(j);
                    yj = obj.goodYArray(j);
                    if ((xi - xj)^2 + (yi - yj))^(1/2) < obj.sailWidth/2
                        numPixNearby = numPixNearby + 1;
                        x(numPixNearby) = xj;
                        y(numPixNearby) = yj;
                    end
                end
                if (numPixNearby < obj.minNumPixInSail)
                    continue;
                end
%                 disp(x(1:numPixNearby));
%                 disp(y(1:numPixNearby));
                
                %Center pix
                gx = x(1:numPixNearby);
                gy = y(1:numPixNearby);
                diag = sqrt((min(gx)-max(gx))^2 + (min(gy)-max(gy))^2);
                centerX = sum(gx)/numPixNearby;
                centerY = sum(gy)/numPixNearby;
                gx = gx - centerX;
                gy = gy - centerY;
                
                % Get bounding box diag
                % 
%                 [xlim, ylim] = boundingbox;
%                 diag = (xlim^2 + ylim^2)^(1/2);
%                bound = polyshape(gx,gy);
%                 plot(bound);
%                 [xlim, ylim] = boundingbox(bound);
% %                 if (size(xlim) == 0) 
% %                     continue; 
% %                 end
%                 %disp(size(xlim(1)));% disp(size(ylim));
%                 diag = sqrt((xlim(1)-xlim(2))^2 + (ylim(1)-ylim(2))^2);
                
                Ixx = gx' * gx;
                Iyy = gy' * gy;
                Ixy = - gx' * gy;
                Inertia = [Ixx Ixy;Ixy Iyy] / numPixNearby;
                % normalized
                lambda = eig(Inertia);
                lambda = sqrt(lambda)*1000.0;
                
                xsum = sum(gx); ysum = sum(gy);
                ixx = sum((gx - xsum).^2);
                iyy = sum((gy - ysum).^2);
                ixy = -sum((gx - xsum).*(gy - ysum));
                thisSailDist = (centerX^2 + centerY^2)^(1/2);
                
                %if (lambda(1) < obj.maxLambda) ...
                if (-1 < obj.maxLambda1 ...
                   && thisSailDist <= closestSailDist)%...
                   %&& abs(obj.sailWidth - diag) <= obj.boundingBoxDiagError)
                   

                    closestSailDist = thisSailDist;
                    sailCenterX = centerX;
                    sailCenterY = centerY;
                    % Compute angle of outward facing normal to line
                    sailTh =  atan2(2*ixy, iyy-ixx)/2.0;
                end
                %break
            end
            % Choose closest detected sail that has
            %  - minimum number of points in circle
            %  - lambda(1) < 1.3,
            %  - bounding box diagonal close to sailwidth?
            % Might need to tune these parameters
            
            
            % If didn't find a sail, dist will still be same as start
            %foundSail = closestSailDist > obj.maxRangeForTarget;
        end
    end
end
