t = 0;

leftArray = zeros(1,1);
rghtArray = zeros(1,1);
posx = 0;
posy = 0;

while(t <= sqrt(32 * pi))
    v = 0.1;
    k = 1/8;
    tread = 0.085;
    angVel = k * t;
    vr = v + (tread / 2) * angVel;
    vl = v - (tread / 2) * angVel;
    dt = 0.001;
    [x, y, th] = modelDiffSteerRobot(vl, vr, t, t+dt*10, dt);
    t = t + dt;
end
