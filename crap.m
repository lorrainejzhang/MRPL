
clear;
clear classes;
sys = mrplSystem(1);
sys.context.robot.forksDown();
sys.context.robot.startLaser();

a = true;    
while a == true
    %a = false;
    range = rangeImage(sys.context.robot);
    scatter(range.xArray, range.yArray)
    [xo, yo, tho] = range.findClosestSail();
    fprintf("xo %f, yo %f, tho %f\n",xo,yo,tho);
    pause(1);

    [x, y, th] = mrplSystem.acquisitionPose(xo, yo, tho, 0.06985, 0.020, 0.0381 - 0.15);
    fprintf("x %f, y %f, th %f\n",x,y,th);
    xi = x*cos(-sys.th1) + y*sin(-sys.th1) + sys.x1;
    yi = -x*sin(-sys.th1) + y*cos(-sys.th1) + sys.y1;
    thi = th + sys.th1;
    sys.executeTrajectoryToAbsPose(xi,yi,thi);
    pause(1);
    clear rangeImage;

    range = rangeImage(sys.context.robot);
    scatter(range.xArray, range.yArray)
    [xo, yo, tho] = range.findClosestSail();
    fprintf("xo %f, yo %f, tho %f\n",xo,yo,tho);
    pause(1);

    [x, y, th] = mrplSystem.acquisitionPose(xo, yo, tho, 0.06985, 0.020, 0.0381);
    fprintf("x %f, y %f, th %f\n",x,y,th);
    xi = x*cos(-sys.th1) + y*sin(-sys.th1) + sys.x1;
    yi = -x*sin(-sys.th1) + y*cos(-sys.th1) + sys.y1;
    thi = th + sys.th1;
    sys.executeTrajectoryToRelativePose(xi,yi,thi);
    pause(1);
    clear rangeImage;

    sys.context.robot.forksUp();
    pause(1);
    sys.context.robot.forksDown();
    pause(1);
    sys.executeTrajectory(trapReferenceControl(true,.05,-1));
    t = trapReferenceControl(false, pi, 1);
    sys.executeTrajectory(t);
    %sys.executeTrajectoryToRelativePose(0, -0.0001, pi);
    pause(10);

end