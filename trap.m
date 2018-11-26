m = mrplSystem(1, 0); % No map!
t = trapReferenceControl(false, pi, 1);
m.executeTrajectory(t,true);