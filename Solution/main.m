% Close and clear everything
rosshutdown
clear
close all
clc

% IP Settings
OwnComputerIP = 'ROS_IP';'192.168.87.106';
OwnComputerHost = '192.168.87.106';
RobotIP = 'http://192.168.87.115:11311';

setenv('ROS_MASTER_URI',RobotIP)
setenv(OwnComputerIP)
rosinit(RobotIP,'NodeHost', OwnComputerHost);

%% Setup
% Subscriptions
cameraSub = rossubscriber('/camera/rgb/image_raw');
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');

% Publishers
[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity');
turtlebot = rospublisher('/mobile_base/commands/velocity');

% Destinations
initialPositionA = [1058 575];
destinationB = [80 200];
destinationC = [600 50];

% Map Definitions
pixelResolution = 1418/61.5;

% Robot definitions
robotWidth = 0.8; %meters

%% From initial position to destinationB
estimatedPosition = DXPathfollowing(initialPositionA, destinationB, robotWidth, pixelResolution);

% Destination Reached find green circle
findGreenCircle(cameraSub, laserSub, velMsg, velPub);

%% Wait
 tic;
 while toc < 2
 end
 
 tic;
 while toc < 3
    velMsg.Linear.X = 0; 
    velMsg.Angular.Z = 1;
    send(robot, velMsg);
 end
 
 tic;
 while toc < 2
    velMsg.Linear.X = 0.6; 
    velMsg.Angular.Z = 0;
    send(robot, velMsg);
 end

%% From destinationB to destinationC
reachedGoal = DXPathfollowing(round(estimatedPosition(1:2)*pixelResolution), destinationC, robotWidth, pixelResolution);

% Destination Reached find green circle
findGreenCircle(cameraSub, laserSub, velMsg, velPub);