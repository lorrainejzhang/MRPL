robot = raspbot();
robot.startLaser();
pause(3);

testdata = cell(10,2);

r8 = .055;
r4 = .0391;
for a = 1:10
    ranges = robot.laser.LatestMessage.Ranges;
    xs = zeros(1,360); ys = zeros(1,360);
    for i = 1:360
            [x, y, th] = irToXy(i, ranges(i));
            d = sqrt(x^2 + y^2);
            trash = false;
            if (d < .1 || d > .95)
                x = 0; y = -r4;
                trash = true;
            end
    %         if ~trash
    %              disp(y)
    %         end

            xs(i) = x; ys(i) = y+r4;
    end
    testdata{a,1} = xs;
    testdata{a,2} = ys;
    beep;
    pause(10);
end
scatter(xs,ys);

disp((ys(1) - ys(6))/2);

robot.stopLaser();