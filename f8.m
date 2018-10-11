global enposx;
global enposy;
global enposth
global oldLeft;
global oldRght;
global oldt;
global robot;
global init;

init = 1;
robot = raspbot();
robot.encoders.NewMessageFcn=@encoderEventListener;


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
end
robot.sendVelocity(0,0);
%disp(i)
%plot(xenArray(1, 3:i), yenArray(1, 3:i));
plot(xenArray(1:i),yenArray(1:i));
aa = xenArray(1:i);
bb = yenArray(1:i);
%xlim([-.6 .6]);
%ylim([-.6 .6]);

%plot(xenArray(1, 1:i), yenArray(1, 1:i));
%plot(xArray(1, 1:i), yArray(1, 1:i));
% xlim([-.6 .6]);
% ylim([-.6 .6]);
% title('cornu');
% xlabel('x');
% ylabel('y');
% axis equal;