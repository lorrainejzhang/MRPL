%thing = mrplSystem.executeTrajectoryToRelativePose(.3048,.3048,0,.2,0);
thing = mrplSystem.executeTrajectoryToRelativePose(-2*.3048,-2*.3048,-pi/2,.2,1);
a = thing.xx;
b = thing.yy;