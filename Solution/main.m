rosshutdown
clear
close all
clc

OwnComputerIP = 'ROS_IP';'192.168.87.106';
OwnComputerHost = '192.168.87.106';
RobotIP = 'http://192.168.87.115:11311';

rosshutdown
setenv('ROS_MASTER_URI',RobotIP)
setenv(OwnComputerIP)
rosinit(RobotIP,'NodeHost', OwnComputerHost);

%% 
% Substribe to odometer and laser scanner. 
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');
[pub,msg] = rospublisher('/mobile_base/commands/reset_odometry','std_msgs/Empty');
send(pub,msg);

% Create publisher for sender velocity commands 
[velPub,velMsg] = ...
    rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');

% Receive laser scan and odometry message.
scanMsg = receive(laserSub);
odompose = odomSub.LatestMessage;
scan = lidarScan(scanMsg); 

% Subscribe to velocity  
robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);

% Destinations
initialPositionA = [1050, 570];
destinationB = [80, 200];
destinationC = [600, 50];

% Map Definitions
pixelToMeterRatio = 1418/53.5;
margin = 0.30;

% Robot definitions
robotWidth = 0.8; %meters
linearVelocity = 0.5;
lookAheadDistance = 0.6;

%% Create Occupancy Map from Shannon map
mapColor = imread('shannon.png');

mapBW = rgb2gray(mapColor);

meanVal = mean(mapBW(:));

imageBinary = mapBW < meanVal;

gridWidth = 25;

grid = flipud(imageBinary);

se = strel('square',gridWidth);
imageBinaryDilated = imdilate(grid,se);
figure(), imshow(imageBinaryDilated);

%% Find Path
dx = DXform(imageBinaryDilated);
dx.plan(destinationB);
path = dx.query(initialPositionA, 'animate'); 
path = path/map.Resolution;

% Set movement properties
goalRadius = 1; 

distanceToGoal = norm(initialPositionA/map.Resolution - goalRadius); 

%% localizationSetup
se = strel('square',gridWidth);
imageBinaryDilated = imdilate(imageBinary,se);
map = robotics.BinaryOccupancyGrid(imageBinaryDilated, 20.88);
startInMeters = initialPositionA/map.Resolution;

initialPose = [startInMeters(1) startInMeters(2) 0]; 

amcl = setupAMCL(map, initialPose);

%% Setup Helper for Vizualization and Driving Turtlebot
visualizationHelper = ExampleHelperAMCLVisualization(map);

%% Set Controller 
controller = robotics.PurePursuit;
controller.Waypoints = path; 
controller.DesiredLinearVelocity = 0.2; 
controller.MaxAngularVelocity = 1; 
controller.LookaheadDistance = 0.2;


% Loop - scan, drive.
i=0;
while(distanceToGoal >= goalRadius)
    % Get turtlebot pose 
    odomdata = receive(odomSub, 2); 
    pose = odomdata.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    %theta = rad2deg(angles(1));
    poseVector = [pose.Position.X+startInMeters(1), pose.Position.Y+startInMeters(2), angles(1)];
    [estimatedPose, estimatedCovariance, i] = runAMCL(amcl, laserSub,poseVector, i, visualizationHelper);
    % Get new values for robot
    [linVel, angVel, targetDir] = controller(estimatedPose);   
    % Pass new information to turtlebot. 
    velmsg.Linear.X = linVel; 
    velmsg.Angular.Z = angVel;       
    send(robot, velmsg); 
    distanceToGoal = norm(poseVector(1:2) - destinationB/map.Resolution);  
end

function [amcl] = setupAMCL(map, initialPose)

odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

% Link: https://se.mathworks.com/help/nav/ref/likelihoodfieldsensormodel.html
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
% Min, Max of sensor readings.
rangeFinderModel.SensorLimits = [0.45 8];
% Occupancy map used for computing likelihood field.
rangeFinderModel.Map = map;

% Transform laser readings from camera frame to base frame of turtlebot. 
% !!! Sensor model is only compatible with sensors fixed on robot's frame
% Query tansformation tree (tf tree) in ROS 
tftree = rostf; 
% Projekteres op til 'verdens koordinatsystem' frem for robottens.
waitForTransform(tftree,'/base_link','/camera_depth_frame');
sensorTransform = getTransform(tftree,'/base_link', '/camera_depth_frame');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

amcl = robotics.MonteCarloLocalization;
amcl.UseLidarScan = true;

% Assign MotionModel and SensorModel 
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

% Update particles when robot's movement exceeds UpdateThresholds. 
% Using larger numbers leads to slower particle depletion at the price of
% slower particle convergence as well. 
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

% Lower and upper bound on number of partiblecs generated during resample process. 
% Increasing increases change of true pose, but impact computation speed. 
amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;
% Initial pose estimate should be obtained according to our setup
% This helper function retrieves thoe robot's current true pose from Gazebo
amcl.InitialPose = initialPose;
amcl.InitialCovariance = eye(3)*0.5;

end 

function [estimatedPose, estimatedCovariance, numberOfIterations] = runAMCL(amcl, laserSub, pose,numberOfIterations, visualizationHelper)
scanMsg = receive(laserSub);
scan = lidarScan(scanMsg);
plot(scan);
[isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, scan);
if isUpdated
    numberOfIterations = numberOfIterations + 1;
    plotStep(visualizationHelper, amcl, estimatedPose, scan, numberOfIterations);
end    
end
