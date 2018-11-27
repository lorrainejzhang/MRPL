% Job scheduler

warning('off', 'all');

% (pickPose, dropPose)
pickAndDropList = [[[1*30.48, 3.5*30.48, 0], [1.75*30.48, 0.5*30.48, pi()]],
                   [[2*30.48, 3.5*30.48, 0], [2.5*30.48, 0.5*30.48, pi()]],
                   [[3*30.48, 3.5*30.48, 0], [3*30.48, 0.5*30.48, pi()]]];

inOrder = true; % pick and drop from order or closest

m = mrplSystem(1, 1); % Uses map!
m.context.robot.startLaser();
pause(.5);

while (~isempty (pickAndDropList))
    currentPose = [m.enposxs(m.i), m.enposys(m.i), m.enposths(m.i)];
    j = 1;
    if (inOrder)
        pickPose, dropPose = pickAndDropList(j);
    else
        closestDistance = poseDistance(currentPose, pickAndDropList(1, 1));
        for i = 2:length(pickAndDropList)
            d = poseDistance(currentPose, pickAndDropList(i, 1));
            if (d < closestDistance)
                closestDistance = d;
                j = i;
            end
        end
        pickPose, dropPose = pickAndDropList(j);
    end
    pickAndDropList(j) = 0; % delete from list
    
    % do pickDropObject stuff!
    
end

m.context.robot.shutdown();
%a = m.ths;
%figure(1);
%plot(m.xs(1:m.i),m.ys(1:m.i),m.enposxs(1:m.i),m.enposys(1:m.i))

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

function d = poseDistance (pose1, pose2)
    x1, y1, th1 = pose1;
    x2, y2, th2 = pose2;
    d = sqrt((x2-x1)^2 + (y2-y1)^2 + (th2-th1)^2);
end
