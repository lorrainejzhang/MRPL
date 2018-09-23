robot = raspbot();

leftStart = robot.encoders.LatestMessage.Vector.X;
rightStart = robot.encoders.LatestMessage.Vector.Y;
avgStart = (leftStart + rightStart) /2;

timeArray = zeros(1,1);
leftArray = zeros(1,1);
rightArray = zeros(1,1);
errorArray = zeros(1,1);

plot(timeArray, leftArray, timeArray, rightArray)
title('Lab1')
xlabel('Time (s)')
ylabel('Distance (m)')
legend('Left Wheel', 'Right Wheel');

kp = 10.0;
kd = 0.03;
ki = 0.0;

oldError = 0;
error = 0;
time = 0;
oldTime = 0;
errorIntegral = 0;

pos = 0;
tic

while(0.1 - pos >= 0.0001 && time < 6)
    oldError = error;
    oldTime = time;
    time = toc;
    pause(.05)
    
    exPos = time * 0.03;
    
    leftPos = robot.encoders.LatestMessage.Vector.X;
    rightPos = robot.encoders.LatestMessage.Vector.Y;
    pos = (((leftPos + rightPos)/2) - avgStart);
    
    error = exPos - pos;
    errorArray = [errorArray, error];
    
    delTime = time - oldTime;
    
    errorDerivative = (error - oldError) / delTime;
    errorIntegral = errorIntegral + error * delTime;
    control = error*kp + errorDerivative*kd + errorIntegral*ki;
    sendVelocity(robot, .03 + control, .03 + control);
    
    timeArray = [timeArray, time];
    rightArray = [rightArray, rightPos - rightStart];
    leftArray = [leftArray, leftPos - leftStart];
    
    plot(timeArray, errorArray);
    title('Lab1');
    xlabel('Time (s)');
    ylabel('Error');
end


robot.stop();
