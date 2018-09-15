robot = raspbot('sim');
changed = 0;
go = true;

leftArray = zeros(1,1);
rightArray = zeros(1,1);

while go
    
    robot.sendVelocity(0.2, 0.2);
    
    leftEncoder = robot.encoders.LatestMessage.Vector.X;
    rghtEncoder = robot.encoders.LatestMessage.Vector.Y;
    time = tic;
    
    changed = 0;
    
    while(not changed)
        robot.encoders.NewMessageFcn=@encoderEventListener;
    end

    % differentiate left and right wheel encoders
    
    timediff = toc(tic);
    leftEncoderNew = robot.encoders.latestMessage.Vector.X;
    rghtEncoderNew = robot.encoders.latestMessage.Vector.Y;
    
    leftVel = (leftEncoderNow - leftEncoder) / timediff;
    rghtVel = (rghtEncoderNow - rghtEncoder) / timediff;
    
    leftArray = [leftArray, leftVel];
    rghtArray = [rghtArray, rghtVel];
    
    changed = false;
    
    plot(leftArray, rghtArray);
    
    pause(0.5);
end
