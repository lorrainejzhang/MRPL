function uref = trapezoidalVelocityProfile(t, sgn)
%uref Return the velocity command of a trapezoidal profile.
% Returns the velocity command of a trapezoidal profile of maximum
% acceleration amax and maximum velocity vmax whose ramps are of
% duration tf. Sgn is the sign of the desired velocities.
% Returns 0 if t is negative. 
vmax = 0.25;
amax = 3 * 0.25;
tramp = vmax/amax;
sf = 1;
tf = (sf + (vmax^2) / amax) / vmax;
if (t < 0)
    uref = 0;
elseif (t < tramp)
    uref = sgn * amax * t;
elseif (t > tramp && t < tf)
    uref = sgn * vmax;
elseif ((tf - t) < tramp)
    uref = sgn * amax * (tf - t);
elseif ((tf - t) < 0)
    uref = 0;
else
    uref = 0;
end
end
