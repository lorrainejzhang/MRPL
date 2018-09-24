robot = raspbot();

leftStart = robot.encoders.LatestMessage.Vector.X;
rightStart = robot.encoders.LatestMessage.Vector.Y;
avgStart = (leftStart + rightStart) /2;

timeArray = zeros(1,1);
leftArray = zeros(1,1);
rightArray = zeros(1,1);
errorArray = zeros(1,1);
urefArray = zeros(1,1);
srefArray = zeros(1,1);
posArray = zeros(1,1);

% plot(timeArray, leftArray, timeArray, rightArray)
% title('Lab1')
% xlabel('Time (s)')
% ylabel('Distance (m)')
% legend('Left Wheel', 'Right Wheel');

kp = 10.0;
kd = 0.03;
ki = 0.0;

oldError = 0;
error = 0;
time = 0;
oldTime = 0;
errorIntegral = 0;
sref = 0;

pos = 0;
tic

while((1 - abs(pos)) >= 0.0001 && time < 6)
    oldError = error;
    oldTime = time;
    time = toc;
    pause(.05)
    
    exPos = time * 0.03;
    
    leftPos = robot.encoders.LatestMessage.Vector.X;
    rightPos = robot.encoders.LatestMessage.Vector.Y;
    pos = (((leftPos + rightPos)/2) - avgStart);
    %disp(pos);
    
    delTime = time - oldTime;
    uref = trapezoidalVelocityProfile(time,1);
    sref = sref + (uref * delTime);
    %disp(sref);
    
    error = exPos - pos;
    errorArray = [errorArray, error];
    urefArray = [urefArray, uref];
    srefArray = [srefArray, sref];
    posArray = [posArray, pos];
    
    
    
    
    errorDerivative = (error - oldError) / delTime;
    errorIntegral = errorIntegral + error * delTime;
    %control = error*kp + errorDerivative*kd + errorIntegral*ki;
    control = uref;
    sendVelocity(robot, control, control);
    
    timeArray = [timeArray, time];
    rightArray = [rightArray, rightPos - rightStart];
    leftArray = [leftArray, leftPos - leftStart];
    
    %plot(timeArray, urefArray);
    title('Lab1');
    xlabel('Time (s)');
    ylabel('Error');
end

plot(timeArray, srefArray, timeArray, posArray);
robot.stop();
