function encoderEventListener(handle,event)
    global enposx;
    global enposy;
    global enposth;
    global oldLeft;
    global oldRght;
    global oldt;
    global robot;
    global goodT;
    global leftFirst;
    global rghtFirst;
    global init;
    
    if init == 1
        leftFirst = robot.encoders.LatestMessage.Vector.X;
        rghtFirst = robot.encoders.LatestMessage.Vector.Y;
        enposx = 0.0;
        enposy = 0.0;
        enposth = 0.0;
        oldLeft = 0.0;
        oldRght = 0.0;
        %oldt = 0.0;
        oldt = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
        goodT = 0;
        
        init = 2;
    end
    
    tread = 0.085;
    encoderDataTimestamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1000000000.0;
    dt = encoderDataTimestamp - oldt;
    %disp(dt);
    %disp(goodT)
    goodT = goodT + dt;
    %disp(goodT)
    oldt = encoderDataTimestamp;
    
    vecx = robot.encoders.LatestMessage.Vector.X;
    vecy = robot.encoders.LatestMessage.Vector.Y;
    dleft = vecx - leftFirst - oldLeft;
    drght = vecy - rghtFirst - oldRght;
    oldLeft = vecx - leftFirst;
    oldRght = vecy - rghtFirst;


%     dleft = robot.encoders.LatestMessage.Vector.X - leftFirst - oldLeft;
%     drght = robot.encoders.LatestMessage.Vector.Y - rghtFirst - oldRght;
%     oldLeft = robot.encoders.LatestMessage.Vector.X - leftFirst;
%     oldRght = robot.encoders.LatestMessage.Vector.Y - rghtFirst;
    
    envl = dleft/dt; 
    envr = drght/dt;
    env = (envl+envr)/2;
    enangVel = (envr-envl)/tread;
    ds = env*dt;
    endth = enangVel*dt/2;
    enposth = enposth + endth;
    endx = cos(enposth)*ds;
    endy = sin(enposth)*ds;
    enposth = enposth + endth;
    
    
    enposx = enposx + endx;
    enposy = enposy + endy;
    %disp(enposx)
    
    pause(.05);
    
end
