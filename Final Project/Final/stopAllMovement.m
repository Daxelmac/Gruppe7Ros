function [] = stopAllMovement(velocityPublisher)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
velocityMessage = rosmessage(velocityPublisher);
velocityMessage.Angular.X = 0; 
velocityMessage.Angular.Y = 0;
velocityMessage.Angular.Z = 0;
velocityMessage.Linear.X = 0;
velocityMessage.Linear.Y = 0;
velocityMessage.Linear.Z = 0;
send(velocityPublisher, velocityMessage);
end

