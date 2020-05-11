function [] = establishConnection(yourIP,turtleIP)
%establishConnection Establish connection to the robot, and call rostopic
%list
%   Detailed explanation goes here
rosshutdown
s = strcat('http://192.168.',turtleIP,':11311');
s1 = strcat('192.168.',yourIP);

setenv('ROS_MASTER_URI',s)
setenv('ROS_IP',s1)
rosinit(s,'NodeHost',s1);

rostopic list
end

