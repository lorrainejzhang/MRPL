%clear;
%clear classes;
sys = mrplSystem(1, 0); % No map!
sys.context.robot.forksDown();
sys.context.robot.startLaser();

a = true;    
%while a == true
for aa = 1 : 2
    a = false;
    range = rangeImage(sys.context.robot);
    scatter(range.xArray, range.yArray)
    [xo, yo, tho] = range.findClosestSail();
    %fprintf("xo %f, yo %f, tho %f\n",xo,yo,tho);
    pause(1);

    %[x, y, th] = mrplSystem.acquisitionPose(xo, yo, tho, 0.06985, 0.020, 0.0381 - 0.15);
    [x, y, th] = mrplSystem.acquisitionPose(xo, yo, tho, .15);
    %x = xo; y = yo; th = tho;
    fprintf("x %f, y %f, th %f\n",x,y,th);
    xi = x*cos(-sys.th1) + y*sin(-sys.th1) + sys.x1;
    yi = -x*sin(-sys.th1) + y*cos(-sys.th1) + sys.y1;
    thi = th + sys.th1;
    sys.executeTrajectoryToAbsPose(xi,yi,thi);
     pause(1);
%     %clear rangeImage;

    range = rangeImage(sys.context.robot);
    scatter(range.xArray, range.yArray)
    [xo, yo, tho] = range.findClosestSail();
    %fprintf("xo %f, yo %f, tho %f\n",xo,yo,tho);
    pause(1);

    [x, y, th] = mrplSystem.acquisitionPose(xo, yo, tho, 0);
    fprintf("x %f, y %f, th %f\n",x,y,th);
    xi = x*cos(-sys.th1) + y*sin(-sys.th1) + sys.x1;
    yi = -x*sin(-sys.th1) + y*cos(-sys.th1) + sys.y1;
    thi = th + sys.th1;
    sys.executeTrajectoryToAbsPose(xi,yi,thi);
    pause(1);
    %clear rangeImage;

    sys.context.robot.forksUp();
    pause(1);
    sys.context.robot.forksDown();
    pause(1);
    %fprintf("x1 %f, y1 %f, th1 %f, offx %f, offy %f, offth %f\n", sys.x1, sys.y1, sys.th1, sys.offx, sys.offy, sys.offth);
    sys.executeTrap(true, .15, -1);
    %fprintf("x1 %f, y1 %f, th1 %f, offx %f, offy %f, offth %f\n", sys.x1, sys.y1, sys.th1, sys.offx, sys.offy, sys.offth);
    sys.executeTrap(false, pi, 1);
    %fprintf("x1 %f, y1 %f, th1 %f, offx %f, offy %f, offth %f\n", sys.x1, sys.y1, sys.th1, sys.offx, sys.offy, sys.offth);

%     t = trapReferenceControl(false, pi, 1);
%     sys.executeTrajectory(t);
%     %sys.executeTrajectoryToRelativePose(0, -0.0001, pi);
    pause(10);

end

sys.context.robot.stopLaser();