function [ x y th] = irToXy( i, r )
    th = (-5*pi/180) + (i - 1) * (pi/180);
    x = r * cos(th);
    y = r * sin(th);
end
