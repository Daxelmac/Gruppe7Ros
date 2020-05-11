function [estimatedPosition, iterations] = runAMCL(amcl, lidarScan, position, iterations, visualizationHelper)
%% Taken from "Localize TurtleBot Using Monte Carlo Localization"
[isUpdated, estimatedPosition, ~] = amcl(position, lidarScan);
if isUpdated
    iterations = iterations + 1;
    plotStep(visualizationHelper, amcl, estimatedPosition, lidarScan, iterations);
end    
end