% Error still in the cm, always a small gap between sref and pos at end?
robot = raspbot();

%1, 0.4, 0.07, 0.2, -1 worked before
%1.5, 0.6, 0.1, 0.2, -1 works well
kp = 3;
kd = 0.05;
ki = 2;
eiMax = 0.2;
sgn = 1;
timestep = 0.05;
tdelay = 0.3;%0.2125;
feedbackOn = 1;

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
runTime = 4.3333;
breakTime = 1;

while (time < runTime + breakTime)
    oldError = error;
    oldTime = time;
    time = toc;
    pause(timestep)
    
    
    
    leftPos = robot.encoders.LatestMessage.Vector.X;
    rightPos = robot.encoders.LatestMessage.Vector.Y;
    pos = (((leftPos + rightPos)/2) - avgStart);
    %disp(pos);
    
    delTime = time - oldTime;
    uref = trapezoidalVelocityProfile(time,sgn);
    udelay = trapezoidalVelocityProfile(time-tdelay,sgn);
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
    if (abs(errorIntegral) > eiMax)
        errorIntegral = sign(errorIntegral) * eiMax;
    end
    control = feedbackOn *(error*kp + errorDerivative*kd + errorIntegral*ki);
    %disp( errorIntegral)
    %control = uref;
    %if (time < runTime)
    sendVelocity(robot, uref+control, uref+control);
    %else
    %    robot.stop();
    %end
    timeArray = [timeArray, time];
    rightArray = [rightArray, rightPos - rightStart];
    leftArray = [leftArray, leftPos - leftStart];
    
    plot(timeArray, errorArray);
    title('Lab1');
    xlabel('Time (s)');
    ylabel('Error');
end
disp(error)

plot(timeArray, srefArray, timeArray, posArray);
robot.stop();
