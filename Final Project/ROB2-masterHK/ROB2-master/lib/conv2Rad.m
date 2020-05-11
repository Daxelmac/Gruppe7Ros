function [rzRad] = conv2Rad(rz)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
degree = conv2Degree(rz);
rzRad = deg2rad(degree); %% Built in function
end

