load ('LAB3_Workspace.mat');
figure, plot(scan);
%%
perpendicular();

%%
establishConnection();

%%
odom = rossubscriber('/odom');
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
start_rz = pose.Orientation.Z
conv2Degree(start_rz)

%%
disp(perpendicular());
%%
laser = rossubscriber('/scan');
scan = receive(laser,3);



