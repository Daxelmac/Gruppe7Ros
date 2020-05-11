function [numberOfIterations] = avoidObstacle(vfh, targetDirection, laserSub, velocityPub, velocityMsg, odomSub, startPositionInMeters, amcl, numberOfIterations, visualizationHelper) 
    % Inspiration from https://www.mathworks.com/help/nav/ug/obstacle-avoidance-with-turtlebot-and-vfh.html
    rate = rateControl(10);

    while rate.TotalElapsedTime < 30
        position = getPosition(odomSub, startPositionInMeters);
        % Get laser scan data
        laserScan = receive(laserSub);
        ranges = double(laserScan.Ranges);
        angles = double(laserScan.readScanAngles);        
 
        % Create a lidarScan object from the ranges and angles
        scan = lidarScan(ranges,angles);   
        
        [estimatedPosition, numberOfIterations] = runAMCL(amcl, scan, position,numberOfIterations, visualizationHelper);
        
        steerDir = vfh(scan, targetDirection-estimatedPosition(3)); 
        figure(65);
        show(vfh);
    
        % Calculate velocities
        if ~isnan(steerDir) % If steering direction is valid
            desiredV = 0.2;
            w = exampleHelperComputeAngularVelocity(steerDir, 0.75);
        else % Stop and search for valid direction
            desiredV = 0.0;
            w = 0.5;
        end

        % Assign and send velocity commands
        velocityMsg.Linear.X = desiredV;
        velocityMsg.Angular.Z = w;
        velocityPub.send(velocityMsg);
    end   
end 
