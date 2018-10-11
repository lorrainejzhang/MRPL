% Code for big integration in constructor function robotTrajectory, takes in referenceControl
t = zeros(obj.numSamples);
v = zeros(obj.numSamples);
w = zeros(obj.numSamples);
p = zeros(obj.numSamples, obj.numSamples, obj.numSamples);
s = zeros(obj.numSamples);
for i = 1: obj.numSamples
    t(i) = (i - 1)*dt;
    vw = referenceControl.computeControl(obj, t(i));
    v(i) = vw(1);
    w(i) = vw(2);
    ds = v(i)*dt; %distance travelled in this dt interval
    s(i + 1) = s(i) + ds;
    p(i + 1, 3) = p(i, 3) + w(i)*dt; %theta_i+1 = theta_i + w*dt
    p(i + 1, 1) = p(i, 1) + cos(p(i + 1, 3))*ds; %x_i+1 = x_i + cos(theta_i+1)*ds
    p(i + 1, 2) = p(i, 2) + sin(p(i + 1, 3))*ds; %y_i+1 = y_i + sin(theta_i+1)*ds
end
t(obj.numSamples) = numSamples*dt;
vw = referenceControl.computeControl(obj, t(obj.numSamples));
v(obj.numSamples) = vw(1);
w(obj.numSamples) = vw(2);

% What pose, vel, angVel, dist functions should look like as class methods?
function pose = getPoseAtTime(obj, t)
    pose = interp1(obj.t, obj.p, t)
end
