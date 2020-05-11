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
vfh.PreviousDirectionWeight = 6;
vfh.CurrentDirectionWeight = 4;
vfh.TargetDirectionWeight = 7;
vfh.SafetyDistance = 0.4;

minRangeObstacleDetect = 0;
maxRangeObstacleDetect = 1;
angleIntervalThreshold = 0.15;

%% Setup Monte Carlo Localization
amcl = setupAMCL(map);

startPositionInMeters = [amcl.InitialPose(1), amcl.InitialPose(2)];

distanceToDestination = norm(startPositionInMeters - endPosition/resolution);

visualizationHelper = ExampleHelperAMCLVisualization(map);

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


