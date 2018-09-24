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

kp = 3.0;
kd = 0.03;
ki = 0.03;

oldError = 0;
error = 0;
time = 0;
oldTime = 0;
errorIntegral = 0;
sref = 0;
sdelay = 0;

pos = 0;
tic
time = 0;

tdelay = .2125;
%while((1 - abs(pos)) >= 0.0001 && time < 6)
while (time < 5.3467)
    oldError = error;
    oldTime = time;
    time = toc;
    pause(.05)
    
    
    
    leftPos = robot.encoders.LatestMessage.Vector.X;
    rightPos = robot.encoders.LatestMessage.Vector.Y;
    pos = (((leftPos + rightPos)/2) - avgStart);
    %disp(pos);
    
    delTime = time - oldTime;
    uref = trapezoidalVelocityProfile(time,1);
    udelay = trapezoidalVelocityProfile(time-tdelay,1);
    sref = sref + (uref * delTime);
    sdelay = sdelay + (udelay * delTime);
    %disp(sref);
    
    exPos = sdelay;
    
    error = exPos - pos;
    errorArray = [errorArray, error];
    urefArray = [urefArray, uref];
    srefArray = [srefArray, sdelay];
    posArray = [posArray, pos];
    
    
    
    
    errorDerivative = (error - oldError) / delTime;
    errorIntegral = errorIntegral + error * delTime;
    if (errorIntegral > 0)
        sign = 1;
    else
        sign = -1;
    end
    eiMax = .01;
    if (abs(errorIntegral) > eiMax)
        errorIntegral = sign * eiMax;
    end
    control = error*kp + errorDerivative*kd + errorIntegral*ki;
    disp( errorIntegral)
    %control = uref;
    sendVelocity(robot, uref+control, uref+control);
    
    timeArray = [timeArray, time];
    rightArray = [rightArray, rightPos - rightStart];
    leftArray = [leftArray, leftPos - leftStart];
    
    plot(timeArray, errorArray);
    title('Lab1');
    xlabel('Time (s)');
    ylabel('Error');
end

%plot(timeArray, srefArray, timeArray, posArray);
robot.stop();
