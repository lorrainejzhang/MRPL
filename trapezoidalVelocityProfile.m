function uref = trapezoidalVelocityProfile(t, sgn)
%uref Return the velocity command of a trapezoidal profile.
% Returns the velocity command of a trapezoidal profile of maximum
% acceleration amax and maximum velocity vmax whose ramps are of
% duration tf. Sgn is the sign of the desired velocities.
% Returns 0 if t is negative. 
vmax = 0.25;
amax = 3 * 0.25;
tramp = vmax/amax; %1/3 meter
tdelay = 0.2125;
sf = 1;
tf = (sf + (vmax^2) / amax) / vmax;
if (t < 0 || t > tf)
    uref = 0;
elseif (t < tramp)
    uref = sgn * amax * t;
elseif (tf - t < tramp)
    uref = sgn * amax * (tf - t);
elseif (tramp < t && t < tf - tramp)
    uref = sgn * vmax;
else
    uref = 0;
end
end
