function encoderEventListener(handle,event)
    global enposx;
    global enposy;
    global enposth;
    global oldLeft;
    global oldRght;
    global oldt;
    global robot;
    global leftFirst;
    global rghtFirst;
    global goodT;
    global offset;
    tread = 0.085;
    encoderDataTimestamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1000000000.0;
    dt = encoderDataTimestamp - oldt;
    %disp(dt);
    %disp(goodT)
    goodT = goodT + dt;
    %disp(goodT)
    oldt = encoderDataTimestamp;
    
    dleft = robot.encoders.LatestMessage.Vector.X - leftFirst - oldLeft;
    drght = robot.encoders.LatestMessage.Vector.Y - rghtFirst - oldRght;
    oldLeft = robot.encoders.LatestMessage.Vector.X - leftFirst;
    oldRght = robot.encoders.LatestMessage.Vector.Y - rghtFirst;
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
    
    %disp(enposx)
    enposx = enposx + endx;
    enposy = enposy + endy;
    %disp(enposx)
    
    pause(.05);
    
end
