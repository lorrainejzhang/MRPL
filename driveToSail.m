clear;
clear classes;
sys = mrplSystem(1);
% sys.context.robot.startLaser()
% pause(1);
% ranges = sys.context.robot.laser.LatestMessage.Ranges;
range = rangeImage(sys.context.robot);
scatter(range.xArray, range.yArray)
% sys.context.robot.stopLaser()
[sailCenterX, sailCenterY, sailTh] = range.findClosestSail();
pause(1);
disp(sailCenterX)
disp(sailCenterY)
disp(sailTh);
sys.executeTrajectoryToAbsPose(sailCenterX, sailCenterY, sailTh);
