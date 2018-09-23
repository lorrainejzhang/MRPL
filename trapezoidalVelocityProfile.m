function uref = trapezoidalVelocityProfile( t , amax, vmax, dist, sgn)
%uref Return the velocity command of a trapezoidal profile.
% Returns the velocity command of a trapezoidal profile of maximum
% acceleration amax and maximum velocity vmax whose ramps are of
% duration tf. Sgn is the sign of the desired velocities.
% Returns 0 if t is negative. 
vmax = 0.25;
amax = 3 * 0.25;
tramp = vmax/amax;
tf = (sf + (vmax^2) / amax) / vmax;
if(t < tramp)
    robot.sendVelocity(amax * t, amax * t);
else if(t > tramp && t < tf)
     robot.sendVelocity(vmax, vmax);
else if ((tf - t) < tramp)
     robot.sendVelocity(amax * (tf - t);
end
