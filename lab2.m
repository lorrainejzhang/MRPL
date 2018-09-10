robot = raspbot();
robot.startLaser();
pause(3);
go = true;

% xArray = zeros(1,1);
% yArray = zeros(1,1);
% thArray = zeros(1,1);
minRange = 1;
minx = 5;
miny = 5;

figure(1); clf;

while(go)
    ranges = robot.laser.LatestMessage.Ranges;
    pause(0.5);
    minRange = 1;
    for i = 1:360
        [x, y, th] = irToXy(i, ranges(i));
        if abs(ranges(i)) <= 1.0 && abs(ranges(i)) >= 0.06 && abs(th) <= pi/2
            if ranges(i) < minRange
                minRange = ranges(i);
                minx = x;
                miny = y;
                axis([-2 2 -2 2]);
                plot(miny, -minx, 'x');
                axis([-2 2 -2 2]);
            end
%             xArray = [xArray, x];
%             yArray = [yArray, y];
%             thArray = [thArray, th];
        end
    end    
    
    pause(0.05);
    
    if minRange >= 1
        robot.sendVelocity(0,0);
    else
        gain = minRange - .5;
        prop = .5;
        vel = prop*gain;
        curv = (miny / (minRange * minRange));
        angVel = curv * vel;
        tread = 0.085;
        rghtVel = vel + (tread/2)*curv;
        leftVel = vel - (tread/2)*curv;
        robot.sendVelocity(leftVel,rghtVel);
    end
   
end
