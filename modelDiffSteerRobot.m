function [x y th] = modelDiffSteerRobot(vl, vr, t0, tf, dt)
    x = 0;
    y = 0;
    th = 0;
    v = (vl + vr) / 2;
    t = t0;
    tread = 0.085;
    while(t < tf)
        angVel = (vr - vl) / tread;
        th = th + angVel * dt / 2;
        x = x + v * cos(th) * dt;
        y = y + v * sin(th) * dt;
        th = th + angVel * dt / 2;
        t = t + dt;
    end
end
