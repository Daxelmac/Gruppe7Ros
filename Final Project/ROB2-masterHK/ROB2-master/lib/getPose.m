function [pose] = getPose(odom)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
end

