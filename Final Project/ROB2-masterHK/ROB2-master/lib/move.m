function [] = move(vel,ang_vel,velpub)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
velmsg = rosmessage(velpub);
velmsg.Linear.X = vel;
velmsg.Angular.Z = ang_vel;
send(velpub,velmsg);
end

