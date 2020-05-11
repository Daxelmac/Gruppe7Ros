function [xPosition, yPosition, yaw] = getRobotPosition(odometrySubscriber)
% Receive odometry data from the turtlebot
odometryData = receive(odometrySubscriber);

% Get the x and y coordinates
position = odometryData.Pose.Pose;
yPosition = position.Position.Y;
xPosition = position.Position.X;

% Calculate yaw angle
quaternion = [position.Orientation.W, position.Orientation.X, position.Orientation.Y, position.Orientation.Z];
euler = quat2eul(quaternion);
yaw = rad2deg(euler(1));

end
