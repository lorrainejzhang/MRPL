sys = mrplSystem(1);
sys.context.robot.startLaser();
robotPose = [15*0.0254,9*0.0254,pi()/2.0];
l = 4*.3048;
lines_p1 = [[0;l] [0;0]];
lines_p2 = [[0;0] [l;0]];
local = lineMapLocalizer(lines_p1,lines_p2,0.3,0.01,0.0005);

fh = gcf;
set(fh,'KeyPressFcn',@keyboardEventListener);



bodyPts = robotModel.bodyGraph();
while (true)
    rangeImagePolar = sys.context.robot.laser.LatestMessage.Ranges;
    xArr = zeros(1,360);
    yArr = zeros(1,360);
    thArr = zeros(1,360);
    
    j = 0;
    for i = 1:360
        r = rangeImagePolar(i);
        [x, y, th] = rangeImage.irToXy(i, r);
        if ~(r < .1 || r > 5*.3048)
            j = j + 1;
            xArr(j) = x;
            yArr(j) = y;
            thArr(j) = th;
            
        end
    end
    
    
    %[success, outPose] = local.refinePose(robotPose,[xArr(1:j); yArr(1:j)],400);
    robotKeypressDriver.drive(sys.context.robot, 1);
    
    outPose = [0,0,pi/2];
    x = outPose(1); y = outPose(2); th = outPose(3);
    xArrWorld = zeros(1,length(xArr));
    yArrWorld = zeros(1,length(xArr));
    for i = 1:length(xArr)
        xTemp = xArr(i);
        yTemp = yArr(i);
        a = xTemp * cos(-th) + yTemp * sin(-th) +x;
        b  = -xTemp * sin(-th) + yTemp * cos(-th) +y;
        xArrWorld(i) =  a;
        yArrWorld(i) = b;
    end
    
%     bx = bodyPts(1); by = bodyPts(2); bth = bodyPts(3);
%     worldBodyPts = pose.bToA()*bodyPts;
    %plot([0,0,l],[l,0,0],'g',worldBodyPts(1,:),worldBodyPts(2,:),'k',xArrWorld,yArrWorld,'ro');  
    plot([0,0,l],[l,0,0],'g',xArrWorld,yArrWorld,'ro'); 
end
