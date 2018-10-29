m = mrplSystem(1);
t = trapReferenceControl(false, pi/2, 1);
m.executeTrajectory(t);