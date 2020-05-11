function [obstacleFound] = checkForObstacles(lidarScan, minRange, maxRange, angleInterval)
    scanLimited = removeInvalidData(lidarScan, 'RangeLimits', [minRange, maxRange], ...
    'AngleLimits', [-angleInterval, angleInterval]); 
    figure(10);
    plot(scanLimited);
    hold on 
    plot(lidarScan);
    hold off
    if (scanLimited.Count == 0) 
        obstacleFound = 0;
    else 
        obstacleFound = 1;
    end
end 