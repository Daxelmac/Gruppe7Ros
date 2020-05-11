function [isObstacle] = checkForObstacle(lidarScan, minRange, maxRange, angleInterval)
    scan = removeInvalidData(lidarScan, 'RangeLimits', [minRange, maxRange], 'AngleLimits', [-angleInterval, angleInterval]); 
    if (scan.Count == 0) 
        isObstacle = 0;
    else 
        isObstacle = 1;
    end
end 
