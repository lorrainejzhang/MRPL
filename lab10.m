sys = mrplSystem(1);
sys.context.robot.startLaser();
robotPose = [15*0.0254,9*0.0254,pi()/2.0];

local = lineMapLocalizer(lines_p1,lines_p2,0.3,0.01,0.0005, [x1;y1]);


p1 = [0 ; l];
p2 = [ 0 ; 0];
p3 = [ l ; 0];
lines_p1 = [p1 p2 p3];
lines_p2 = [p2 p3 p1];

while (true)
    rangeImagePolar = sys.context.robot.laser.LatestMessage.Ranges;
    xArr = zeroes(1,360);
    yArr = zeroes(1,360);
    thArr = zeroes(1,360);
    
    for i = 1:360
        [x, y, th] = rangeImage.irToXy(i, r);
        xArr(i) = x;
        yArr(i) = y;
        thArr(i) = th;
    end
    
    xArr(xArr > .1) = []; yArr(yArr > .1) = [];
    [success, outPose] = local.refinePose(robotPose,[xArr; yArr],400)
    
    
end
