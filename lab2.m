robot = raspbot();
robot.startLaser();
pause(3);
go = true;

% xArray = zeros(1,1);
% yArray = zeros(1,1);
% thArray = zeros(1,1);
minRange = 1; %m
minx = 5; %m
miny = 5; %m

figure(1); clf;

while(go)
    ranges = robot.laser.LatestMessage.Ranges;
    pause(0.5);
    minRange = 1;
    for i = 1:360
        [x, y, th] = irToXy(i, ranges(i)); % th ranges from 0 to 2pi
        if abs(ranges(i)) <= 1.0 && abs(ranges(i)) >= 0.06 && ((0 <= th && th <= pi/2) || (3*pi/2 <= th && th <= 2*pi))
            if ranges(i) < minRange
                minRange = ranges(i);
                minx = x;
                miny = y;
                axis([-2 2 -2 2]);
                plot(miny, -minx, 'x');
                title('Lab2 (robot perspective)');
                axis([-2 2 -2 2]);
                xlabel('x (m)');
                ylabel('y (m)');
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
        gain = 0.5;
        vel = gain * (minRange - .5);
        curv = miny / minRange^2;
        angVel = curv * vel;
        tread = 0.085; %m
        leftVel = vel - (tread/2)*angVel;
        rghtVel = vel + (tread/2)*angVel;
        robot.sendVelocity(leftVel,rghtVel);
    end
   
end
