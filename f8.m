global enposx;
global enposy;
global enposth
global oldLeft;
global oldRght;
global oldt;
global robot;
global leftFirst;
global rghtFirst;

enposx = 0.0;
enposy = 0.0;
enposth = 0.0;
oldLeft = 0.0;
oldRght = 0.0;
oldt = 0.0;
robot = raspbot();
robot.encoders.NewMessageFcn=@encoderEventListener;
leftFirst = robot.encoders.LatestMessage.Vector.X;
rghtFirst = robot.encoders.LatestMessage.Vector.Y;

xArray = zeros(1,300000);
yArray = zeros(1,300000);

xenArray = zeros(1, 600000);
yenArray = zeros(1, 600000);
% myPlot = plot(xenArray, yenArray, 'b-');
% xlim([-.6 .6]);
% ylim([-.6 .6]);



v = 0.2;
sf = 1;
tf = sf/v;
kth = 2*pi/sf;
kk = 15.1084;

ks = 3;
Tf = ks*tf;
tread = 0.085;
posx = 0;
posy = 0;
posth = 0;
posenx = 0;
poseny = 0;
posenth = 0;

i = 0;
told = 0;
T = 0;
t = 0;
Ti = tic();
while(T <= Tf)
    T = toc(Ti);
    told = t;
    t = T/ks;
    
    s = v*t;
    curv = (kk/ks) * sin(kth*s);
    angVel = curv*v;
    vr = v + (tread / 2) * angVel;
    vl = v - (tread / 2) * angVel;
    robot.sendVelocity(vl, vr);
    
    i = i + 1;
    xenArray(i) = enposx;
    yenArray(i) = enposy;
    pause(.05);
    
    %leftOld = robot.encoders.LatestMessage.Vector.X;
    %rghtOld = robot.encoders.LatestMessage.Vector.Y;
    %time = tic;
    
    
%     changed = 0;
%     
%     while(changed == 0)
%         
%         pause(.1);
%         %disp("here")
%     end

    % differentiate left and right wheel encoders
        
%     [endx, endy, endth] = modelDiffSteerRobot(envl, envr, t-timediff, t, timediff/10); % local trajectory est.
%     enr = sqrt(endx^2 + endy^2); % have to change it from robot coords to our plot
%     posenth = posenth + endth; % since it changed direction
%     posenx = posenx + cos(posenth)*enr;
%     poseny = poseny + sin(posenth)*enr;
    
    
%     plot(xenArray(1, 1:i), yenArray(1, 1:i));
%     xlim([-.6 .6]);
%     ylim([-.6 .6]);
    %set(myPlot, 'xdata', [get(myPlot,'xdata') xenArray], 'ydata', [get(myPlot,'ydata') yenArray]);
    
    %plot(xenArray(1:i
    
%     [dx, dy, dth] = modelDiffSteerRobot(vl, vr, told, t, (t - told)/10); % local trajectory est.
%     r = sqrt(dx^2 + dy^2); % have to change it from robot coords to our plot
%     posth = posth + dth; % since it changed direction
%     posx = posx + cos(posth)*r;
%     posy = posy + sin(posth)*r;
%     
%     %i = i + 1;
%     xArray(i) = posx;
%     yArray(i) = posy;
    
   
    
    
end
robot.sendVelocity(0,0);
disp(i)
plot(xenArray(1, 3:i), yenArray(1, 3:i));
xlim([-.6 .6]);
ylim([-.6 .6]);

%plot(xenArray(1, 1:i), yenArray(1, 1:i));
%plot(xArray(1, 1:i), yArray(1, 1:i));
% xlim([-.6 .6]);
% ylim([-.6 .6]);
% title('cornu');
% xlabel('x');
% ylabel('y');
% axis equal;