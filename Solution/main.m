% Close and clear everything
rosshutdown
clear
close all
clc

% IP Settings
OwnComputerIP = 'ROS_IP';'192.168.7.25';
OwnComputerHost = '192.168.7.25';
RobotIP = 'http://192.168.7.41:11311';

setenv('ROS_MASTER_URI',RobotIP)
setenv(OwnComputerIP)
rosinit(RobotIP,'NodeHost', OwnComputerHost);

%% Setup
% Subscriptions
cameraSub = rossubscriber('/camera/rgb/image_raw');
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');

% Publishers
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
turtlebot = rospublisher('/mobile_base/commands/velocity');

% Map Definitions
pixelResolution = 1418/61.5; % Dimensions of image used divided with location measurements

% Destinations
initialPositionA = [1058 575];
destinationB = [110 230];
destinationC = [600 75];

% Robot definitions
robotWidth = 0.8; %meters
maxAngularVelocity = 1;

%% From initial position A to destination B
[amcl, visualizationHelper] = DXPathfollowing(laserSub, odomSub, turtlebot, initialPositionA, destinationB, robotWidth, pixelResolution);

% Destination reached, now find green circle
locateGreenMarker(cameraSub, laserSub, velMsg, velPub, robotWidth);
disp('First green circle found')

currentPosition = getCurrentPosition(odomSub, (initialPositionA / pixelResolution));
% New scan for position
scanMsg = receive(laserSub);
scan = lidarScan(scanMsg);
[estimatedPosition, ~] = updateAndPlotAMCL(amcl, scan, currentPosition, 0, visualizationHelper);

%% From destination B to destination C
[amcl, visualizationHelper] = DXPathfollowing(laserSub, odomSub, turtlebot, round((estimatedPosition(1:2) * pixelResolution)), destinationC, robotWidth, pixelResolution);

% Destination Reached find green circle
locateGreenMarker(cameraSub, laserSub, velMsg, velPub, robotWidth);

disp('Second green circle found')