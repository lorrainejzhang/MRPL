sys = mrplSystem(1, 1); % Uses map!
sys.context.robot.startLaser();
outPose = [15*0.0254,9*0.0254,pi()/2.0];
outPose = [.3048,.3048,pi/2];
l = 4*.3048;
lines_p1 = [[0;l] [0;0]];
lines_p2 = [[0;0] [l;0]];
local = lineMapLocalizer(lines_p1,lines_p2,0.3,0.01,0.0005);

driver = robotKeypressDriver(gcf);



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
        if ~(r < .06 || r > 5*.3048)
            j = j + 1;
            xArr(j) = x;
            yArr(j) = y;
            thArr(j) = th;
            
        end
    end
    
    gx = xArr(1:15:j);
    gy = yArr(1:15:j);
    
    [success, outPose] = local.refinePose(outPose,[gx; gy],50);
    driver.drive(sys.context.robot, 1);
    
    %outPose = [0,0,pi/2];
    x = outPose(1); y = outPose(2); th = outPose(3);
    xArrWorld = zeros(1,length(gx));
    yArrWorld = zeros(1,length(gx));
    for i = 1:length(gx)
        xTemp = gx(i);
        yTemp = gy(i);
        a = xTemp * cos(-th) + yTemp * sin(-th) +x;
        b  = -xTemp * sin(-th) + yTemp * cos(-th) +y;
        xArrWorld(i) =  a;
        yArrWorld(i) = b;
    end
    
%     bx = bodyPts(1); by = bodyPts(2); bth = bodyPts(3);
%     worldBodyPts = pose.bToA()*bodyPts;
    %plot([0,0,l],[l,0,0],'g',worldBodyPts(1,:),worldBodyPts(2,:),'k',xArrWorld,yArrWorld,'ro');  
    plot([0,0,l],[l,0,0],'g',xArrWorld,yArrWorld,'ro'); 
    pause(.2);
end
