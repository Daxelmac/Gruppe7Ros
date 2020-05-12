function [amclToReturn, visualizationHelper] = DXPathfollowing(laserSub, odomSub, turtlebot, startPosition, endPosition, robotWidth, pixelResolution)
    %Publishers
    [resetOdomPub, resetOdomMsg] = rospublisher('/mobile_base/commands/reset_odometry','std_msgs/Empty');
    send(resetOdomPub, resetOdomMsg); % resetting odometery

    [velTwistPub, velTwistMsg] = rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');

    velTurtlebotMsg = rosmessage(turtlebot);

    %% Settings
    goalRadius = 1;
    margin = 0.3;

    %% Create Occupancy Map from Shannon map
    mapColor = imread('shannon.png');

    mapBW = rgb2gray(mapColor);

    meanVal = mean(mapBW(:));

    imageBinary = mapBW < meanVal;

    imageBinaryFlipped = flipud(imageBinary);

    se = ones(round(((robotWidth / 2) + margin) * pixelResolution));

    imageBinaryDilatedFlipped = imdilate(imageBinaryFlipped, se);
    %figure(), imshow(imageBinaryDilatedFlipped);

    map = robotics.BinaryOccupancyGrid(imageBinary, pixelResolution);
    %figure(), show(map)

    %% Find best path using D-star
    dx = DXform(imageBinaryDilatedFlipped);
    dx.plan(endPosition);
    path = dx.query(startPosition);
    path = path / pixelResolution;

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
    vfh.PreviousDirectionWeight = 2; % Value found with trial and error
    vfh.CurrentDirectionWeight = 4; % Value found with trial and error
    vfh.TargetDirectionWeight = 7; % Value found with trial and error
    vfh.SafetyDistance = 0.5;

    lowerRangeLimit = 0;
    upperRangeLimit = 1; % Value found with trial and error
    angleThreshold = 0.15; % Value found with trial and error

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

    distanceToDestination = norm(startPositionInMeters - (endPosition / pixelResolution));
    %% Drive to destination
    iterations = 0;
    while(distanceToDestination >= goalRadius)
        currentPosition = getCurrentPosition(odomSub, startPositionInMeters);

        % New scan for obstacles
        scanMsg = receive(laserSub);
        scan = lidarScan(scanMsg);

        % Find the newest estimated position
        [estimatedPosition, iterations] = updateAndPlotAMCL(amcl, scan, currentPosition, iterations, visualizationHelper);

        obstacleInFront = checkForObstacle(scan, lowerRangeLimit, upperRangeLimit, angleThreshold);

        if(obstacleInFront)
            % Obstacle found
            iterations = avoidObstacle(vfh, estimatedPosition(3), laserSub, turtlebot, velTwistMsg, odomSub, startPositionInMeters, amcl, iterations, visualizationHelper);
        else % No Obstacles
            [linearVel, angularVel, ~] = controller(estimatedPosition);
            velTurtlebotMsg.Angular.Z = angularVel;
            velTurtlebotMsg.Linear.X = linearVel; 
            send(turtlebot, velTurtlebotMsg);
        end 
        % Find the new estimated distance to the destination
        distanceToDestination = norm(estimatedPosition(1:2) - endPosition / pixelResolution);  
    end
    % When the while-loop is done, the destination should have been reached
    disp("Destination Reached")
    amclToReturn = amcl;
end