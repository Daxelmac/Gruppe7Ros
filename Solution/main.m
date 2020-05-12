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

% Map Definitions
pixelResolution = 1418/61.5;

% Destinations
initialPositionA = [1058 575];
destinationB = [110 230];
destinationC = [600 75];
initialPositionAMeters = initialPositionA/pixelResolution;

% Robot definitions
robotWidth = 0.8; %meters
maxAngularVelocity = 1;

%% From initial position to destinationB
estimatedPosition = DXPathfollowing(initialPositionA, destinationB, robotWidth, pixelResolution);

% Destination Reached find green circle
findGreenCircle(cameraSub, laserSub, velMsg, velPub, robotWidth);

disp('First green circle found')

%% From destinationB to destinationC
DXPathfollowing(destinationB, destinationC, robotWidth, pixelResolution);

% Destination Reached find green circle
findGreenCircle(cameraSub, laserSub, velMsg, velPub, robotWidth);

disp('Second green circle found')
