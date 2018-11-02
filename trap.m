m = mrplSystem(1);
t = trapReferenceControl(false, pi, 1);
m.executeTrajectory(t,true);