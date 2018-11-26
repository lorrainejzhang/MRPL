clear;
clear classes;
sys = mrplSystem(1, 0); % No map!
sys.context.robot.startLaser()
for i = 1:1
    
    % pause(1);
    % ranges = sys.context.robot.laser.LatestMessage.Ranges;
    
    range = rangeImage(sys.context.robot);
    scatter(range.xArray, range.yArray)
    % sys.context.robot.stopLaser()
    [xo, yo, tho] = range.findClosestSail();
    fprintf("xo %f, yo %f, tho %f\n",xo,yo,tho);
    pause(1);
%     disp(x)
%     disp(sailCenterY)
%     disp(sailTh);
    %sys.context.robot.forksDown();
    [x, y, th] = mrplSystem.acquisitionPose(xo, yo, tho, 0.06985 ,0.020, 0.0381);
    fprintf("x %f, y %f, th %f\n",x,y,th);
    disp(sqrt((x-xo)^2+(y-yo)^2));
    %x = xo; y = yo; th = tho;
%     sys.executeTrajectoryToAbsPose(x, y, th);
    xi = x*cos(-sys.th1) + y*sin(-sys.th1) + sys.x1;
    yi = -x*sin(-sys.th1) + y*cos(-sys.th1) + sys.y1;
    thi = th + sys.th1;
    %sys.context.robot.forksDown();
    sys.executeTrajectoryToAbsPose(xi,yi,thi);
    %sys.context.robot.forksUp();
    pause(1);
    clear rangeImage;
end
