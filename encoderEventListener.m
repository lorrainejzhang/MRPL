function encoderEventListener(handle,event)
    global changed;
    changed = 1;
    encoderDataTimestamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1000000000.0;
    %disp("hi")
    % encoderFrame = encoderFrame + 1;
    
  
    
end
