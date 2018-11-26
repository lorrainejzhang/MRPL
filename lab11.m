warning('off', 'all');

m = mrplSystem(1, 1); % Uses map!
m.context.robot.startLaser();
pause(.5);

m.executeTrajectoryToAbsPose(.3048, 0.9144, pi()/2.0);
m.executeTrajectoryToAbsPose(0.9144, 0.3048, 0.0);
m.executeTrajectoryToAbsPose(0.6096, 0.6096, pi()/2.0);
%m.executeTrajectoryToAbsPose(-.3048, .3048, pi);
m.context.robot.shutdown();
%a = m.ths;
%figure(1);
%plot(m.xs(1:m.i),m.ys(1:m.i),m.enposxs(1:m.i),m.enposys(1:m.i))
term = ((m.xs(m.i) - m.enposxs(m.i))^2 + (m.ys(m.i) - m.enposys(m.i))^2)^.5

% figure(2);
% plot(m.ts(1:m.i),m.ths(1:m.i),m.ts(1:m.i),m.enposths(1:m.i))
% x = m.xs(1:m.i); y = m.ys(1:m.i);
% pause(4);
% m1 = mrplSystem(1);
% m1.executeTrajectoryToAbsPose(.3048, .3048, 0);
% m1.executeTrajectoryToAbsPose(-.3048, -.3048, -pi/2);
% m1.executeTrajectoryToAbsPose(0, 0, pi/2);
% m1.context.robot.shutdown();
% figure(2);
% plot(m1.xs(1:m1.i),m1.ys(1:m1.i),m1.enposxs(1:m1.i),m1.enposys(1:m1.i))
% 
% % figure(4);
% % plot(m1.ts(1:m1.i),m1.ths(1:m1.i),m1.ts(1:m1.i),m1.enposths(1:m1.i))
% x1 = m1.xs(1:m1.i); y1 = m1.ys(1:m1.i);
% figure(5);
% plot(m.xs(1:m.i)-m1.xs(1:m1.i),m1.ys(1:m1.i));
