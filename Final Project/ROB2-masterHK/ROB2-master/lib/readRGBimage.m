function [img] = readRGBimage()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if ismember('/camera/rgb/image_color/compressed',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_color/compressed');
end
if ismember('/camera/rgb/image_raw',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_raw');
end
imgraw = receive(imsub); % a serialised image
img = readImage(imgraw); % decode image
end

