xArray = zeros(1,300000);
yArray = zeros(1,300000);

v = 0.2;
sf = 1;
tf = sf/v;
kth = 2*pi/sf;
kk = 15.1084;

ks = 1;
Tf = ks*tf;
tread = 0.085;
posx = 0;
posy = 0;
posth = 0;

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
    
    [dx, dy, dth] = modelDiffSteerRobot(vl, vr, told, t, (t - told)/10); % local trajectory est.
    r = sqrt(dx^2 + dy^2); % have to change it from robot coords to our plot
    posth = posth + dth; % since it changed direction
    posx = posx + cos(posth)*r;
    posy = posy + sin(posth)*r;
    
    i = i + 1;
    xArray(i) = posx;
    yArray(i) = posy;
end

plot(xArray(1, 1:i), yArray(1, 1:i));
% xlim([0 0.4]);
% ylim([0 0.4]);
% title('cornu');
% xlabel('x');
% ylabel('y');
% axis equal;