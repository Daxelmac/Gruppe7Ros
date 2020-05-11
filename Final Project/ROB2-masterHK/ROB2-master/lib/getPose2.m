function [xPose, yPose, yaw] = getPose2(odomsub)
% Receive odometry information from turtlebot and return x and y coordinate
% and yaw angle
%	Yaw angle in degrees

% Receive odometry data from turtlebot
odomdata = receive(odomsub);

% Select x and y coordinates
pose = odomdata.Pose.Pose;
xPose = pose.Position.X;
yPose = pose.Position.Y;

% Calculate yaw angle from quaternion data
quat = [pose.Orientation.W, pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z];
eul = quat2eul(quat);
yaw = rad2deg(eul(1));

end

