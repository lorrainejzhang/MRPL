% Job scheduler

warning('off', 'all');
% pickAndDropList = {[3.5*.3048, -1*.3048, 0], [0.5*.3048, -1.75*.3048, pi]};

pickAndDropList = {[3.5*.3048, -1*.3048, 0], [0.5*.3048, -1.75*.3048, pi], ...
                   [3.5*.3048, -2*.3048, 0], [0.5*.3048, -2.5*.3048, pi], ...
                   [3.5*.3048, -3*.3048, 0], [0.5*.3048, -3*.3048, pi]};

% pickAndDropList = {[3.5*.3048, -1*.3048, 0], [0.5*.3048, -0.75*.3048, pi], ...
%                    [3.5*.3048, -2*.3048, 0], [0.5*.3048, 0.75*.3048, pi], ...
%                    [3.5*.3048, -3*.3048, 0], [0.5*.3048, 0.75*.3048, pi]};

%inOrder = true; % pick and drop from order or closest

m = mrplSystem(1, 0);
m.context.robot.startLaser();
pause(.5);
%m.executeTrap(false, pi, 1); % Spin around
for i = 1:6
    pose = pickAndDropList{i};
    x0 = pose(1);
    y0 = pose(2);
    th0 = pose(3);
    if mod(i, 2) == 1
        m.pickDropObject(true, x0, y0, th0);
    else
        m.pickDropObject(false, x0, y0, th0);
    end
end
m.context.robot.stopLaser();
m.context.robot.shutdown();
% 
% pickPose = zeros(1,3);
% dropPose = zeros(1,3);
% x0 = 0;
% y0 = 0;
% th0 = 0;
% x1 = 0;
% y1 = 0;
% th1 = 0;
% 
% while (~isempty (pickAndDropList))
%     currentPose = [m.x1, m.y1, m.th1];
%     j = 1;
%     if (inOrder)
%     else
%         closestDistance = poseDistance(currentPose, pickAndDropList(1, 1));
%         for i = 2:length(pickAndDropList)
%             d = poseDistance(currentPose, pickAndDropList(i, 1));
%             if (d < closestDistance) && mod(i, 2) == 1
%                 closestDistance = d;
%                 j = i;
%             end
%         end
%         
%     end
%     pose = pickAndDropList(j);
%     disp(pose);
%     %pickAndDropList(j) = 0; % delete from list
%     x0 = pose(1);
%     y0 = pose(2);
%     th0 = pose(3);
%     if mod(j, 2) == 1
%         m.pickDropObject(true, x0, y0, th0);
%     else
%         m.pickDropObject(false, x0, y0, th0);
%     end
%     
% end
% 
% m.context.robot.shutdown();
% %a = m.ths;
% %figure(1);
% %plot(m.xs(1:m.i),m.ys(1:m.i),m.enposxs(1:m.i),m.enposys(1:m.i))
% 
% % figure(2);
% % plot(m.ts(1:m.i),m.ths(1:m.i),m.ts(1:m.i),m.enposths(1:m.i))
% % x = m.xs(1:m.i); y = m.ys(1:m.i);
% % pause(4);
% % m1 = mrplSystem(1);
% % m1.executeTrajectoryToAbsPose(.3048, .3048, 0);
% % m1.executeTrajectoryToAbsPose(-.3048, -.3048, -pi/2);
% % m1.executeTrajectoryToAbsPose(0, 0, pi/2);
% % m1.context.robot.shutdown();
% % figure(2);
% % plot(m1.xs(1:m1.i),m1.ys(1:m1.i),m1.enposxs(1:m1.i),m1.enposys(1:m1.i))
% % 
% % % figure(4);
% % % plot(m1.ts(1:m1.i),m1.ths(1:m1.i),m1.ts(1:m1.i),m1.enposths(1:m1.i))
% % x1 = m1.xs(1:m1.i); y1 = m1.ys(1:m1.i);
% % figure(5);
% % plot(m.xs(1:m.i)-m1.xs(1:m1.i),m1.ys(1:m1.i));
% 
% function d = poseDistance (pose1, pose2)
%     x1, y1, th1 = pose1;
%     x2, y2, th2 = pose2;
%     d = sqrt((x2-x1)^2 + (y2-y1)^2 + (th2-th1)^2);
% end
