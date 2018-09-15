robot = raspbot();
global changed;
go = true;

leftArray = zeros(1,1);
rghtArray = zeros(1,1);

while go
    
    robot.sendVelocity(0.2, 0.2);
    
    leftEncoder = robot.encoders.LatestMessage.Vector.X;
    rghtEncoder = robot.encoders.LatestMessage.Vector.Y;
    time = tic;
    
    
    changed = 0;
    
    while(changed == 0)
        robot.encoders.NewMessageFcn=@encoderEventListener;
        pause(.1);
        disp("here")
    end

    % differentiate left and right wheel encoders
    
    timediff = toc(tic);
    leftEncoderNew = robot.encoders.LatestMessage.Vector.X;
    rghtEncoderNew = robot.encoders.LatestMessage.Vector.Y;
    
    leftVel = (leftEncoderNew - leftEncoder) / timediff;
    rghtVel = (rghtEncoderNew - rghtEncoder) / timediff;
    disp(leftVel)
    
    leftArray = [leftArray, leftVel];
    rghtArray = [rghtArray, rghtVel];
    
    changed = 0;
    
    plot(rghtArray);
    
    pause(0.5);
end
