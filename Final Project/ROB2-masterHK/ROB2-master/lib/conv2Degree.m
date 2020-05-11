function [degree] = conv2Degree(rz)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if(rz<=0)
   degree =-(180)*rz;
end
if(rz>0)
   rz=1-rz;
   degree =(180)*rz+ 180; 
end
end

