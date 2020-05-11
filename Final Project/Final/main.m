rosshutdown
clear
close all
clc

% Make connection to robot
OwnComputerIP = 'ROS_IP';'192.168.7.25';
OwnComputerHost = '192.168.7.25';
RobotIP = 'http://192.168.7.41:11311/';

setenv('ROS_MASTER_URI', RobotIP)
setenv(OwnComputerIP)
rosinit(RobotIP, 'NodeHost', OwnComputerHost);

% Definitions
robotWidth = 0.8; %meters
pixelResolution = 2000/53.5;
margin = 0.30;
monteCarloCounter = 0;

% Destinations
initialPositionA = [39.5, 24];
destinationB = [3.5, 7.5];
destinationC = [22.5, 2];

% Robot settings
linearVelocity = 0.5;
lookAheadDistance = 0.6;

% Obstacle detection
minObstacleDistance = 0;
maxObstacleDistance = 1.2;
angleLimitsInterval = 0.15;

% Objects for different subscriptions in ROS
odom = rossubscriber('/odom');
odomSub = rossubscriber('/odom');
resetOdometry = rospublisher('/mobile_base/commands/reset_odometry');

imSub = rossubscriber('/camera/rgb/image_raw');
laserSub = rossubscriber('/scan');

[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');

% Map creation
map_color = imread('ShannonMap.png');

mapBW = rgb2gray(map_color);
% imshow(map_bw)

meanVal = mean(mapBW(:));

% Converting the image to binary
imageBinary = mapBW<meanVal;
%imshow(im_binary);
%imageBinary = rot90(imageBinary ,3); %REMOVE LATER
%imageBinary = flip(imageBinary);%REMOVE LATER
%imageBinary = fliplr(imageBinary);%REMOVE LATER

% Dilation
wallOnes = ones(round(((robotWidth / 2) + margin) * pixelResolution));
imageBinaryDilated = imdilate(imageBinary, wallOnes);
%imshow(im_binary_dilated)

% Invert binary map
map = binaryOccupancyMap(imageBinaryDilated, pixelResolution);
mapNoDilation = binaryOccupancyMap(imageBinary, pixelResolution);
show(map)
%% Matlab Probabilistic Roadmaps
%close all

pathPlanner = mobileRobotPRM(map, 750);
%show(pathPlanner)
pathFromAToB = findpath(pathPlanner, initialPositionA, destinationB);
figure(10)
show(pathPlanner)
pathFromBToC = findpath(pathPlanner, destinationB, destinationC);
figure(11)
show(pathPlanner)

% Reset odometry (pure persuit)
resetMessage = rosmessage(resetOdometry);
send(resetOdometry ,resetMessage);
pause(2)

%% Monte Carlo - Taken from "Localize TurtleBot Using Monte Carlo Localization" example
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = mapNoDilation;

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

visualizationHelper = ExampleHelperAMCLVisualization(mapNoDilation);

%% VFH
% https://www.mathworks.com/help//ros/ug/obstacle-avoidance-with-turtlebot-and-vfh.html
vfh = controllerVFH;
vfh.UseLidarScan = true;
% Set this weight to find most optimal path. 

% Limit for range readings. Used to ignore obstacles that are too far from
% the vehicle. 
vfh.DistanceLimits = [0.05 1];

% Radius of actual robot! 
vfh.RobotRadius = 0.4;

vfh.MinTurningRadius = 0.1;

% Meters around the robot which should be avoided! 
vfh.SafetyDistance = 0.5;
%% Pure pursuit
controller = controllerPurePursuit('DesiredLinearVelocity', linearVelocity, 'LookaheadDistance', lookAheadDistance, 'MaxAngularVelocity', 2, 'Waypoints', pathFromAToB);
i = 0; % Instantiate vector field histograms

% Set goal radius
goalRadius = 0.1;
robotInitialLocation = [amcl.InitialPose(1), amcl.InitialPose(2)];
robotGoal = destinationB;
distanceToGoal = norm(robotInitialLocation - robotGoal);

%% Get to point B
while(distanceToGoal > goalRadius)
    % Receive laser scan and odometry message
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    
    % Create lidarScan object to pass to the AMCL object
    scan = lidarScan(scanMsg);
    
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    
    pose = [odompose.Pose.Pose.Position.X + amcl.InitialPose(1), odompose.Pose.Pose.Position.Y + amcl.InitialPose(2), odomRotation(1)];
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, scan);
	
    obstacleFound = checkForObstacles(scan, minObstacleDistance, maxObstacleDistance, angleLimitsInterval);
    if(obstacleFound)
        [steerDir, numberOfIterations] = avoidObstacle(vfh, ...
            estimatedPose(3), 10, laserSub, velPub, velMsg,...
            odomSub, initialPositionA, amcl,i, visualizationHelper);
        monteCarloCounter = monteCarloCounter + 1;
    else
        %Starts a new montocarlo
		%amcl = monteCarloInit(mapNoDilation);
		monteCarloCounter = 0;
    end

	% Scanning and avoiding objects
	%objectAvoided = avoidObject(laserSub,odomSub,velPub,scan);

   % if(~objectAvoided)
	%	monteCarloCounter = monteCarloCounter + 1;
%	else

 %   end
    % Calculation distance to goal
    %distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-B);
    % Asking controller 1 for linear velocity and angular velocity
    [vel,ang_vel] = controller([estimatedPose(1), estimatedPose(2), estimatedPose(3)]);
    % Tells the robot to move
    %disp(angles)
    move(vel,ang_vel,velPub);
    
    %disp (pose(1))
    %pause(1)
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
	if (isUpdated && monteCarloCounter > 3)
         i = i + 1;
         plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
	end
	
	distanceToGoal = norm([estimatedPose(1), estimatedPose(2)] - robotGoal);
end
disp("Destination B reached")
% Find first green circle
spinToLocateGreenCircle(12, imSub, odomSub, velPub);

allignGreenCircle(12, 3, imSub, odomSub, velPub);

aoa = driveTowardsGreenCircle(laserSub, velPub, odomSub);

allignGreenCircle(4, 1, imSub, odomSub, velPub);

allignToWall(laserSub, odomSub, velPub);

rotateDegree2(180,1,false,odomSub,velPub)
disp("Circel reached")

% From B to C
controller.Waypoints = pathFromBToC;
amcl = monteCarloInit(mapNoDilation);

robotInitialLocation = [amcl.InitialPose(1), amcl.InitialPose(2)];
robotGoal = destinationC;
distanceToGoal = norm(robotInitialLocation - robotGoal);

while(distanceToGoal > goalRadius)
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);
    
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
  

    
    pose = [odompose.Pose.Pose.Position.X + amcl.InitialPose(1), odompose.Pose.Pose.Position.Y + amcl.InitialPose(2), odomRotation(1)];
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);
	
	% Scanning and avoiding objects
	objectAvoided = avoidObject(laserSub,odomSub,velPub,scan);

    if(~objectAvoided)
		monteCarloCounter = monteCarloCounter + 1;
	else
		%Starts a new montocarlo
		amcl = monteCarloInit(mapNoDilation);
		monteCarloCounter = 0;
    end
    % Calculation distance to goal
    %distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-B);
    % Asking controller 1 for linear velocity and angular velocity
    [vel,ang_vel] = controller([estimatedPose(1), estimatedPose(2), estimatedPose(3)]);
    % Tells the robot to move
    %disp(angles)
    move(vel,ang_vel,velPub);

    %disp (pose(1))
    %pause(1)
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
	if (isUpdated && monteCarloCounter > 3)
         i = i + 1;
         plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
	end
	
	distanceToGoal = norm([estimatedPose(1), estimatedPose(2)] - robotGoal);
end
disp("Destination C reached")
% Find first green circle
spinToLocateGreenCircle(12, imSub, odomSub, velPub);

allignGreenCircle(12, 3, imSub, odomSub, velPub);

aoa = driveTowardsGreenCircle(laserSub, velPub, odomSub);

allignGreenCircle(4, 1, imSub, odomSub, velPub);

allignToWall(laserSub, odomSub, velPub);

disp("Circel reached")

%% % MOved it to a function

while(1)
	move(0.2,0,velPub);
	scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
	objectAvoided = avoidObject(laserSub,odomSub,velPub,scan);
end