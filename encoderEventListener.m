function encoderEventListener(handle,event)
    persistent changed
    encoderDataTimestamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1000000000.0;
    % encoderFrame = encoderFrame + 1;
    changed = 1;
end
