rosshutdown
clear
close all
clc

% Definitions
robotWidth = 0.8; %meters
pixelResolution = (2445-8)/53.5;
margin = 0.30;
monteCarloCounter = 0;

% Destinations
A = [46, 27];
B = [2, 9];
C = [26, 3];
establishConnection('7.25', '7.41');
% Making objects for subscribing and publishing the topics
odom = rossubscriber('/odom');
resetOdometry = rospublisher('/mobile_base/commands/reset_odometry');
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');
imSub = rossubscriber('/camera/rgb/image_raw');
[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');
%startup_rvc
%%
% Loading occupancy map of Shannon
% Load Color Image
%map_color = imread('World_Simple.png');
%figure(1), imshow(map_color)

% Convert to Grayscale
%map_bw = rgb2gray(map_color);
map_bw = imread('shannonMap.tif');
%figure(1), imshow(map_bw)

% Calculating Mean Pixel Value
mean_value = mean(map_bw(:));

% Converting the image to Binary using threshold
im_binary = map_bw<mean_value;
%figure(), imshow(im_binary);
im_binary = rot90(im_binary,3);
%figure(), imshow(im_binary)
im_binary = flip(im_binary);
im_binary = fliplr(im_binary);
%figure(), imshow(im_binary)
% Making dilation on walls
SE = ones(round(((robotWidth/2)+margin)*pixelResolution));
im_binary_dilated = imdilate(im_binary,SE);
%figure(), imshow(im_binary_dilated)
% Invert binary

map = binaryOccupancyMap(im_binary_dilated,pixelResolution);
map_without_dilate = binaryOccupancyMap(im_binary,pixelResolution);
%figure(), show(map)
%% Matlab PRM - better than Corke
close all
tic
prmComplex = mobileRobotPRM(map,500);
%show(prmComplex)
path1 = findpath(prmComplex,A,B);
figure(10)
show(prmComplex)
path2 = findpath(prmComplex,B,C);
figure(11)
show(prmComplex)
toc

%% PRM - Corke
%Add probabilistic roadmapping
% figure
% imagesc(im_binary_dilated), colorbar
% hold on, plot(A(1), A(2), 'r*'), text(A(1), A(2), 'A')
% hold on, plot(B(1), B(2), 'r*'), text(B(1), B(2), 'B')
% 
% prm = PRM(im_binary_dilated)
% prm.plan('npoints', 300)  % planning
% prm.plot() 
% 
% path = prm.query(A, B) % waypoints
% prm.plot()
% path = path/Map_pixel_m_resolution;

%%
% Reset odometry (pure persuit)
reset_msg = rosmessage(resetOdometry);
send(resetOdometry,reset_msg);
pause(2)

%amcl.release()

% Creating array for logging the position of the robots movement - from
% its perspective 
logarray = zeros(100,3); %(pure persuit)
index = 1;
%% Monte Carlo - 
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map_without_dilate;
% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
waitForTransform(tftree,'/base_link','/camera_depth_frame');
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
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose 
amcl.InitialCovariance = eye(3)*0.5;
visualizationHelper = ExampleHelperAMCLVisualization(map_without_dilate); % Start a figure for amcl to print points in. PlotStep() later.
%wanderHelper = ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);
%% Pure persuit
controller = controllerPurePursuit('DesiredLinearVelocity',0.5,'LookaheadDistance',0.6,'MaxAngularVelocity',2,'Waypoints',path1);
amcl = monteCarloInit(map_without_dilate);
visualizationHelper = ExampleHelperAMCLVisualization(map_without_dilate); % Start a figure for amcl to print points in. PlotStep() later.
%amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
i = 0; % Instantiate vector field histograms

% Set goal radius
goalRadius = 0.1;
robotInitialLocation = [amcl.InitialPose(1), amcl.InitialPose(2)];
robotGoal = B;
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
		amcl = monteCarloInit(map_without_dilate);
		monteCarloCounter = 0;
    end
    % Calculation distance to goal
    %distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-B);
    % Asking controller 1 for linear velocity and angular velocity
    [vel,ang_vel] = controller([estimatedPose(1), estimatedPose(2), estimatedPose(3)]);
    % Tells the robot to move
    %disp(angles)
    move(vel,ang_vel,velPub);
    % Logging data and incrementing index
    logarray(index,1) = estimatedPose(1)-odompose.Pose.Pose.Position.X + amcl.InitialPose(1);
    logarray(index,2) = estimatedPose(2)-odompose.Pose.Pose.Position.Y + amcl.InitialPose(2);
    logarray(index,3) = estimatedPose(3)-odomRotation(1) + amcl.InitialPose(3);
    index = index + 1;

    
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
controller.Waypoints = path2;
amcl = monteCarloInit(map_without_dilate);

robotInitialLocation = [amcl.InitialPose(1), amcl.InitialPose(2)];
robotGoal = C;
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
		amcl = monteCarloInit(map_without_dilate);
		monteCarloCounter = 0;
    end
    % Calculation distance to goal
    %distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-B);
    % Asking controller 1 for linear velocity and angular velocity
    [vel,ang_vel] = controller([estimatedPose(1), estimatedPose(2), estimatedPose(3)]);
    % Tells the robot to move
    %disp(angles)
    move(vel,ang_vel,velPub);
    % Logging data and incrementing index
    logarray(index,1) = estimatedPose(1)-odompose.Pose.Pose.Position.X + amcl.InitialPose(1);
    logarray(index,2) = estimatedPose(2)-odompose.Pose.Pose.Position.Y + amcl.InitialPose(2);
    logarray(index,3) = estimatedPose(3)-odomRotation(1) + amcl.InitialPose(3);
    index = index + 1;

    
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