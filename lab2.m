robot = raspbot();
robot.startLaser();
pause(3);
go = true;

xArray = zeros(1,1);
yArray = zeros(1,1);
thArray = zeros(1,1);

while(go)
    ranges = robot.laser.LatestMessage.Ranges;
    pause(0.5);
    for i = 1:360
        [x, y, th] = irToXy(i, ranges(i));
        if abs(ranges(i)) <= 1 && abs(ranges(i)) >= 0.06
            xArray = [xArray, x];
        end
        yArray = [yArray, y];
        thArray = [thArray, th];
        plot(xArray);
    end
end
