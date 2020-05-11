rosshutdown
close all

%% 
% !!! REMEMBER TO CHANGE IP BASED ON HOST !!!
setenv('ROS_MASTER_URI','http://192.168.80.128:11345')
setenv('ROS_IP','192.168.1.107')
rosinit('http://192.168.80.128:11311','NodeHost','192.168.1.107');


start = [1105 567]; 
goal = [150 200];

pixelToMeterRatio = 22.5;

estimatedPose = DStarWithObstacleAvoidance(start, goal, pixelToMeterRatio, 0);

goal = [670 70];
%%
estimatedPose = DStarWithObstacleAvoidance(...
    round(estimatedPose(1:2)*pixelToMeterRatio), ...
    goal, pixelToMeterRatio, estimatedPose(3))
