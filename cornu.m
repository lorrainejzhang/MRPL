t = 0;
maxt = sqrt(32*pi);
i = 1;
dt = 0.001;
maxi = ceil(maxt/dt) + 1;

xArray = zeros(1,maxi);
yArray = zeros(1,maxi);

v = 0.1;
k = 1/8; % can play with this to see how curve changes
tread = 0.085;
posx = 0;
posy = 0;
posth = 0;

i = 1;
while(t <= maxt)
    i = i + 1;
    angVel = k * t;
    vr = v + (tread / 2) * angVel;
    vl = v - (tread / 2) * angVel;
    [dx, dy, dth] = modelDiffSteerRobot(vl, vr, t, t+10*dt, dt); % local trajectory est.
    r = sqrt(dx^2 + dy^2); % have to change it from robot coords to our plot
    posth = posth + dth; % since it changed direction
    posx = posx + cos(posth)*r;
    posy = posy + sin(posth)*r;
    xArray(i) = posx;
    yArray(i) = posy;
    t = t + 10*dt;
end

plot(xArray(1, 1:i), yArray(1, 1:i));
xlim([0 0.4]);
ylim([0 0.4]);
title('cornu');
xlabel('x');
ylabel('y');
axis equal;
