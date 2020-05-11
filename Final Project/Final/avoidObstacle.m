function [steerDir, numberOfIterations] = avoidObstacle(controllerVFH, targetDir,...
length, laserSub, velPub, velMsg, odomSub, startInMeters, amcl, ...
numberOfIterations, visualizationHelper) 
    % https://www.mathworks.com/help//nav/ref/ratecontrol.html
    % Loop frequency. 
    rate = rateControl(10);

    while rate.TotalElapsedTime < length
        % Updating pose for acml
        poseVector = getPoseVector(odomSub, startInMeters);
        % Get laser scan data
        laserScan = receive(laserSub);
        ranges = double(laserScan.Ranges);
        angles = double(laserScan.readScanAngles);        
 
        % Create a lidarScan object from the ranges and angles
        scan = lidarScan(ranges,angles);   
        
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, scan);
        
        steerDir = controllerVFH(scan, targetDir-estimatedPose(3)); 
        figure(4);
        show(controllerVFH);
    
        % Calculate velocities
        if ~isnan(steerDir) % If steering direction is valid
            desiredV = 0.2;
            w = exampleHelperComputeAngularVelocity(steerDir, 0.5);
        else % Stop and search for valid direction
            desiredV = 0.0;
            w = 0.5;
        end

        % Assign and send velocity commands
        velMsg.Linear.X = desiredV;
        velMsg.Angular.Z = w;
        velPub.send(velMsg);
    end   
end 