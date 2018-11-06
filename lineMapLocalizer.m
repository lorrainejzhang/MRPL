classdef lineMapLocalizer < handle
 %mapLocalizer A class to match a range scan against a map in
 % order to find the true location of the range scan relative to
 % the map.

     properties(Constant)
        maxErr = 0.05; % 5 cm
        minPts = 5; % min # of points that must match
     end

     properties(Access = private)
     end

     properties(Access = public)
         lines_p1 = [];
         lines_p2 = [];
         gain = 0.3;
         errThresh = 0.01;
         gradThresh = 0.0005;
         ptsInRangeImage;
     end
     
     methods
     
         function obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
             % create a lineMapLocalizer
             obj.lines_p1 = lines_p1;
             obj.lines_p2 = lines_p2;
             obj.gain = gain;
             obj.errThresh = errThresh;
             obj.gradThresh = gradThresh;
         end
         
         function [x, y, th, success] = main(obj, maxIters, pose)
             options.maxIterations=maxIters;
             [pose,~,~,exitflag] = lsqnonlin(@obj.transformedError, pose, [-inf,-inf,-inf], [inf,inf,inf], options);
             success = (exitflag == 1);
             x = pose(1); y = pose(2); th = pose(3);
         end
         
         
        function error = transformedError(obj, pose)
            transformed = obj.transform(pose);
            error = obj.closestDistanceToLines(transformed);
            error(error == Inf) = 0;
            %disp(transformed);
        end
        
        function transformed = transform(obj, pose)
            x = pose(1);
            y = pose(2);
            th = pose(3);
            transformed = obj.ptsInRangeImage;
            %disp(transformed);
            for i = 1:length(transformed)
                xTemp = transformed(1,i);
                yTemp = transformed(2,i);
                %disp(xTemp); disp(yTemp);
                a = xTemp * cos(th) - yTemp * sin(th) +x;
                b  = xTemp * sin(th) + yTemp * cos(th)+y;
                transformed(1,i) =  a;
                transformed(2,i) = b;
            end
        end
         
        function ro2 = closestSquaredDistanceToLines(obj,pi)
        % Find the squared shortest distance from pi to any line
        % segment in the supplied list of line segments.
        % pi is an array of 2d points
        % throw away homogenous flag
            pi = pi(1:2,:);
            r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,...
                obj.lines_p1(:,i),obj.lines_p2(:,i));
            end
            ro2 = min(r2Array,[],1);
        end
        
        function ids = throwOutliers(obj,pose,ptsInModelFrame)
        % Find ids of outliers in a scan.
            worldPts = pose.bToA()*ptsInlFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            ids = find(r2 > lineMapLocalizer.maxErr*lineMapLocalizer.maxErr);
        end
        
        function avgErr2 = fitError(obj,pose,ptsInModelFrame)
        % Find the variance of perpendicular distances of
        % all points to all lines
        % transform the points
            worldPts = pose.bToA()*ptsInModelFrame;

            r2 = obj.closestSquaredDistanceToLines(worldPts);
            r2(r2 == Inf) = [];
            err2 = sum(r2);
            num = length(r2);
            if(num >= lineMapLocalizer.minPts)
                avgErr2 = err2/num;
            else
            % not enough points to make a guess
                avgErr2 = inf;
            end
        end
        
        function ro2 = closestDistanceToLines(obj,pi)
            squared = obj.closestSquaredDistanceToLines(pi);
            ro2 = sqrt(squared);
        end

     function [success, outPose] = refinePose(obj,inPose,ptsInModelFrame,maxIters)
            obj.ptsInRangeImage = ptsInModelFrame;
            [x, y, th, success] = obj.main(maxIters, inPose);
            outPose = [x,y,th];
      end
        
     end
     

end
