test = load('testdata');
b = test.testdata;

xs = b{6,1}; ys = b{6,2};
ri = rangeImage(xs,ys);
scatter(xs,ys);
[cx,cy,th] = ri.findClosestSail()

% for i = 1:10
%     %figure(i);
%     scatter(b{i,1},b{i,2});
%     pause(3);
% end