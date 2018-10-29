function [xs, ys, ths] = lab8(robot)

    pause(3);

    r8 = .055;
    r4 = .0391 - 0.0292;
    r4 = .0412;
%     r4 = 0;
        %disp(robot)
    ranges = robot.laser.LatestMessage.Ranges;
    xs = zeros(1,360); ys = zeros(1,360); ths = zeros(1,360);
    for i = 1:360
            [x, y, th] = irToXy(i, ranges(i));
            d = sqrt(x^2 + y^2);
            if (d < .1 || d > .95)
                x = 0; y = -r4;
            end

            xs(i) = x; ys(i) = y + r4; ths(i) = th;
    end


    scatter(xs,ys);
