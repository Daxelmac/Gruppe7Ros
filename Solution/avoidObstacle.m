function [iterations] = avoidObstacle(vfh, targetDirection, laserSub, turtlebot, velTwistMsg, odomSub, startPositionInMeters, amcl, iterations, visualizationHelper) 
    % Inspiration from https://www.mathworks.com/help/nav/ug/obstacle-avoidance-with-turtlebot-and-vfh.html
    rate = rateControl(10);

    while rate.TotalElapsedTime < 30
        currentPosition = getCurrentPosition(odomSub, startPositionInMeters);
        
        % Get laser scan data
        laserScan = receive(laserSub);
        ranges = double(laserScan.Ranges);
        angles = double(laserScan.readScanAngles);
 
        % Create a lidarScan object from the ranges and angles
        scan = lidarScan(ranges,angles);
        
        [estimatedPosition, iterations] = updateAndPlotAMCL(amcl, scan, currentPosition, iterations, visualizationHelper);
        
        % Call VFH object to computer steering direction
        steerDir = vfh(scan, targetDirection - estimatedPosition(3));
    
        % Calculate velocities
        if ~isnan(steerDir) % If steering direction is valid
            desiredV = 0.2;
            w = exampleHelperComputeAngularVelocity(steerDir, 0.75); % Value found with trial and error
        else % Stop and search for valid direction
            desiredV = 0.0;
            w = 0.5;
        end

        % Assign and send velocity commands
        velTwistMsg.Linear.X = desiredV;
        velTwistMsg.Angular.Z = w;
        turtlebot.send(velTwistMsg);
    end   
end 