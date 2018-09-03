robot = raspbot('Raspbot-0');

leftStart = robot.encoders.LatestMessage.Vector.X;
rightStart = robot.encoders.LatestMessage.Vector.Y;
avgStart = (leftStart + rightStart) /2;

timeArray = zeros(1,1);
leftArray = zeros(1,1);
rightArray = zeros(1,1);

pos = 0;
tic
while(pos < .3048)
    sendVelocity(robot, .05, .05)
    curr = toc;
    pause(.05)
    
    leftPos = robot.encoders.LatestMessage.Vector.X;
    rightPos = robot.encoders.LatestMessage.Vector.Y;
    pos = (((leftPos + rightPos)/2) - avgStart);
    
    timeArray = [timeArray, curr];
    rightArray = [rightArray, rightPos - rightStart];
    leftArray = [leftArray, leftPos - leftStart];
    
    plot(timeArray, leftArray, timeArray, rightArray)
end

tic
while(toc <= 1)
    sendVelocity(robot, 0, 0)
    pause(.05)
end

leftStart = robot.encoders.LatestMessage.Vector.X;
rightStart = robot.encoders.LatestMessage.Vector.Y;
avgStart = (leftStart + rightStart) /2;

old = curr;
leftPos = leftStart;
rightPos = rightStart;

pos = 0;
tic
while(abs(pos) < .3048)
    sendVelocity(robot, -.05, -.05)
    curr = toc;
    pause(.05)
    
    leftPos = robot.encoders.LatestMessage.Vector.X;
    rightPos = robot.encoders.LatestMessage.Vector.Y;
    pos = (((leftPos + rightPos)/2) - avgStart);
    
    timeArray = [timeArray, old + curr];
    rightArray = [rightArray, rightPos - rightStart];
    leftArray = [leftArray, leftPos - leftStart];
    
    plot(timeArray, leftArray, timeArray, rightArray)
end
%disp(pos)

% tic
% while(toc <= 1)
%     sendVelocity(robot, 0, 0)
%     pause(.05)
% end
% 
% tic
% while(toc <= 5)
%     sendVelocity(robot, -.03, -.03)
%     pause(.05)
% end

sendVelocity(robot, 0, 0)