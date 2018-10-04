global enposx;
global enposy;
global enposth;
global oldLeft;
global oldRght;
global oldt;
global robot;
global leftFirst;
global rghtFirst;
global goodT;
global offset;

enposx = 0.0;
enposy = 0.0;
enposth = 0.0;
oldLeft = 0.0;
oldRght = 0.0;
oldt = 0.0;
robot = raspbot();
%offset = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
goodT = 0;
robot.encoders.NewMessageFcn=@encoderEventListener;
leftFirst = robot.encoders.LatestMessage.Vector.X;
rghtFirst = robot.encoders.LatestMessage.Vector.Y;

ref = figure8ReferenceControl(3, 1, .5);
%ref = trapReferenceControl();
traj = robotTrajectory(100000, ref);
follower = trajectoryFollower();

%plot(traj.t,traj.th)
s = traj.s;
ths = traj.th;

size = 100000;
dur = ref.getTrajectoryDuration;
dt = dur/size;
% ts = zeros(1,size);
xs = zeros(1,size);
ys = zeros(1,size);
enposxs = zeros(1,size);
enposys = zeros(1,size);
% vs = zeros(1,size);
tread = 0.085;
i = 1;
T = 0;
Ti = tic();
while(T <= dur + 1)
    T = toc(Ti);

    %ts(i) = (i-1)*dt;
    [x, y, th] = traj.getPoseAtTime(T);
    %disp(enposx)
    follower.sendVel(robot, T, traj, enposx,enposy,enposth, goodT, ref);
    
    if abs(x) > 1
        x = 0;
    end
    if abs(y) > 1
        y = 0;
    end
    xs(i) = x; ys(i) = y;
    enposxs(i) = enposx; enposys(i) = enposy;
    i = i + 1;
    pause(.05);
end
robot.sendVelocity(0,0);
robot.shutdown();
xint = traj.x;
%plot(xs,ys)
plot(xs,ys,enposxs,enposys)
