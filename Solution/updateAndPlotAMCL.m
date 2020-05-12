function [estimatedPosition, iterations] = updateAndPlotAMCL(amcl, lidarScan, currentPosition, iterations, visualizationHelper)
%% Taken from "Localize TurtleBot Using Monte Carlo Localization"
[isUpdated, estimatedPosition, ~] = amcl(currentPosition, lidarScan);
if(isUpdated)
    iterations = iterations + 1;
    plotStep(visualizationHelper, amcl, estimatedPosition, lidarScan, iterations);
end    
end