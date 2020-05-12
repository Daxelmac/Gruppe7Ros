function [estimatedPosition] = DXPathfollowing(start, stop, robotWidth, resolution)
% Substriptions
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');

% Publishers
[publisher,msg] = rospublisher('/mobile_base/commands/reset_odometry','std_msgs/Empty');
send(publisher,msg); % reseting odometery

[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');
turtlebot = rospublisher('/mobile_base/commands/velocity');

velmsg = rosmessage(turtlebot);

%% Settings
goalRadius = 1; 
startPosition = start;
endPosition = stop;
pixelResolution = resolution;
margin = 0.3;

%% Create Occupancy Map from Shannon map
mapColor = imread('shannon.png');

mapBW = rgb2gray(mapColor);

meanVal = mean(mapBW(:));

imageBinary = mapBW < meanVal;

imageBinaryFlipped = flipud(imageBinary);

se = ones(round(((robotWidth / 2) + margin) * pixelResolution));

imageBinaryDilatedFlipped = imdilate(imageBinaryFlipped,se);
figure(), imshow(imageBinaryDilatedFlipped);

map = robotics.BinaryOccupancyGrid(imageBinary, pixelResolution);
figure(), show(map)

%% Find Path
dx = DXform(imageBinaryDilatedFlipped);
dx.plan(endPosition);
path = dx.query(startPosition); 
path = path/resolution;
%% Purepursuit controller
controller = controllerPurePursuit;

controller.Waypoints = path;

controller.DesiredLinearVelocity = 0.4;

controller.MaxAngularVelocity = 1;

controller.LookaheadDistance = 1;

%% VFH Controller for obstacle avoidance 
% Inpiration from https://www.mathworks.com/help/nav/ug/obstacle-avoidance-with-turtlebot-and-vfh.html
vfh = controllerVFH;
vfh.UseLidarScan = true;
vfh.DistanceLimits = [0.05 1];
vfh.RobotRadius = robotWidth/2;
vfh.MinTurningRadius = 0.2;
vfh.PreviousDirectionWeight = 2;
vfh.CurrentDirectionWeight = 4;
vfh.TargetDirectionWeight = 7;
vfh.SafetyDistance = 0.5;

minRangeObstacleDetect = 0;
maxRangeObstacleDetect = 1;
angleIntervalThreshold = 0.15;

%% Setup Monte Carlo Localization 
% Taken from "Localize TurtleBot Using Monte Carlo Localization"
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map;

% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
waitForTransform(tftree, '/base_link', '/camera_depth_frame');
sensorTransform = getTransform(tftree,'/base_link', '/camera_depth_frame');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;
amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
amcl.InitialCovariance = eye(3) * 0.5;

visualizationHelper = ExampleHelperAMCLVisualization(map);

startPositionInMeters = [amcl.InitialPose(1), amcl.InitialPose(2)];

distanceToDestination = norm(startPositionInMeters - endPosition/resolution);
%% Drive to destination

i=0;
while(distanceToDestination >= goalRadius)
    position = getPosition(odomSub, startPositionInMeters);
    scanMsg = receive(laserSub);
    scan = lidarScan(scanMsg);
    [estimatedPosition, i] = runAMCL(amcl, scan, position, i, visualizationHelper);
    
    isObstacle = checkForObstacle(scan, minRangeObstacleDetect, maxRangeObstacleDetect, angleIntervalThreshold);

    if(isObstacle)
        % Obstacle found
        numberOfIterations = avoidObstacle(vfh, estimatedPosition(3), laserSub, turtlebot, velMsg, odomSub, startPositionInMeters, amcl,i, visualizationHelper);
        i = numberOfIterations;
    else % No Obstacles
        [linVel, angVel, ~] = controller(estimatedPosition); 
        velmsg.Linear.X = linVel; 
        velmsg.Angular.Z = angVel;
        send(turtlebot, velmsg);
    end 
            
    distanceToDestination = norm(estimatedPosition(1:2) - endPosition/resolution);  
end
disp("Destination Reached")


