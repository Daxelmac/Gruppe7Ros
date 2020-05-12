function [obstacleInFront] = checkForObstacle(lidarScan, lowerRangeLimit, upperRangeLimit, angleThreshold)
    % Inspiration from https://se.mathworks.com/help/robotics/ref/removeinvaliddata.html
    validScan = removeInvalidData(lidarScan, 'RangeLimits', [lowerRangeLimit, upperRangeLimit], 'AngleLimits', [-angleThreshold, angleThreshold]); 
    if (validScan.Count == 0) 
        obstacleInFront = false;
    else 
        obstacleInFront = true;
    end
end