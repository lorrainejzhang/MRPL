figure(1);
mrplSystem.executeTrajectoryToRelativePose(0.3048, 0.3048, 0, 0.2, 0); clear;
pause(0.05);
figure(2);
mrplSystem.executeTrajectoryToRelativePose(-0.6096, -0.6096, -pi()/2.0, 0.2, 0); clear;
pause(0.05);
figure(3);
mrplSystem.executeTrajectoryToRelativePose(-0.3048, 0.3048, pi()/2.0, 0.2, 0); clear;
