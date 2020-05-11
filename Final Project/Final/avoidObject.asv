function [obstacleAvoided] = avoidObject(laserSubscriber, odometrySubscriber, velocityPublisher, lidarScan)

%Setup of variables
objectBlocking = false;
offSetFromStartCounter = 0;
maxDistanceToObstacleMiddle = 0.65;
maxDistanceToObstacleSides = 0.79;
maxDistanceToObstacleFarSides = 0.45;
farLeftDistanceValues = [620:640];
farRightDistanceValues = [1:21];
parallelMoveSize = 0.3;
stepSize = 0.21;
angleVelocityInSteps = 1;
linearVelocityInSteps = 0.3;
obstacleAvoided = false;
sensorCenterDistanceValues = [300:340];
rightDistanceValues = [90:113];
leftDistanceValues = [497:520];

% Actual function:
    % Make sure to omitnan to avoid errors
    distanceToObstacle = mean(lidarScan.Ranges(sensorCenterDistanceValues), 'omitnan');
    if(isObstacleInTheWay(distanceToObstacle, maxDistanceToObstacleMiddle, maxDistanceToObstacleSides, rightDistanceValues, leftDistanceValues, lidarScan))
		stopAllMovement(velocityPublisher);
        % Should be placed elsewhere
        obstacleAvoided = true;
        objectBlocking = true;
        firstTimeParallelFlag = true;  
        pause(2);
        % Takes a new scan
        scanMessage = receive(laserSubscriber);
        lidarScan = lidarScan(scanMessage);
        % Calculates new distance to the obstacle
        distanceToObstacle = mean(lidarScan.Ranges(sensorCenterDistanceValues), 'omitnan');
        disp("Distance to obstacle: " + distanceToObstacle);
            while(offSetFromStartCounter ~= 0 || objectBlocking == true)
                while(isObstacleInTheWay(distanceToObstacle, maxDistanceToObstacleMiddle, maxDistanceToObstacleSides, rightDistanceValues, leftDistanceValues, lidarScan))
                        distanceObstacleLeftSide = mean(lidarScan.Ranges(farLeftDistanceValues),'omitnan');
                        distanceObstacleRightSide = mean(lidarScan.Ranges(farRightDistanceValues),'omitnan');
                        
                        % If NaNs are detected mean returns NaN.
                        % If NaNs to the right and to the left, prefer going left
                        distanceObstacleLeftSide(isnan(distanceObstacleLeftSide)) = 1;
                        distanceObstacleRightSide(isnan(distanceObstacleRightSide)) = 0;
                        if(distanceObstacleLeftSide > distanceObstacleRightSide)
                            whichWayToTurn = 1;
                            offSetFromStartCounter = offSetFromStartCounter -1;
                        else
                            whichWayToTurn = -1;
                            offSetFromStartCounter = offSetFromStartCounter +1;
                        end
                    rotateSpecificDegrees(90 * whichWayToTurn, angleVelocityInSteps * whichWayToTurn, odometrySubscriber, velocityPublisher);
                    moveSpecificDistance(stepSize, linearVelocityInSteps, velocityPublisher, odometrySubscriber);
                    % Rotate back to previous angle
                    rotateSpecificDegrees(-90 * whichWayToTurn, -angleVelocityInSteps * whichWayToTurn, odometrySubscriber, velocityPublisher);
                    % Takes a new scan
                    scanMessage = receive(laserSubscriber);
                    lidarScan = lidarScan(scanMessage);
                end
                objectBlocking = false;

                % Now, maybe parallel with the object
                while(offSetFromStartCounter ~= 0 && objectBlocking == false)
                    % Takes a new scan
                    scanMessage = receive(laserSubscriber);
                    lidarScan = lidarScan(scanMessage);
                    if(isObstacleInTheWayWithFarSides(sensorCenterDistanceValues, leftDistanceValues, farRightDistanceValues, maxDistanceToObstacleMiddle, maxDistanceToObstacleSides, maxDistanceToObstacleFarSides, rightDistanceValues, farLeftDistanceValues))
                        objectBlocking = false;
                        % Drives forward
                        moveSpecificDistance(parallelMoveSize, linearVelocityInSteps, velocityPublisher, odometrySubscriber);
                        if(~firstTimeParallelFlag)    
                            rotateSpecificDegrees(-90 * whichWayToTurn, -angleVelocityInSteps * whichWayToTurn, odometrySubscriber, velocityPublisher);
                            % Takes a new scan
                             scanMessage = receive(laserSubscriber);
                             lidarScan = lidarScan(scanMessage);
                             distanceToObstacle = mean(lidarScan.Ranges(sensorCenterDistanceValues), 'omitnan');
                             if(isObstacleInTheWay(distanceToObstacle, maxDistanceToObstacleMiddle, maxDistanceToObstacleSides, rightDistanceValues, leftDistanceValues, lidarScan))
                                 rotateSpecificDegrees(90 * whichWayToTurn, angleVelocityInSteps * whichWayToTurn, odometrySubscriber, velocityPublisher);
                             else
                                 while(offSetFromStartCounter ~= 0 && ~(mean(lidarScan.Ranges(sensorCenterDistanceValues),'omitnan')<maxDistanceToObstacleMiddle || mean(lidarScan.Ranges(leftDistanceValues),'omitnan')<maxDistanceToObstacleSides || mean(lidarScan.Ranges(leftDistanceValues),'omitnan')<maxDistanceToObstacleSides))
                                     moveRelative(stepSize,linearVelocityInSteps,velocityPublisher,odometrySubscriber);
                                     % Takes a new scan
                                    scanMessage = receive(laserSubscriber); lidarScan = lidarScan(scanMessage);
                                    offSetFromStartCounter = offSetFromStartCounter + whichWayToTurn;
                                    %disp(offsetcounter);
                                 end
                                rotateDegree2(90*whichWayToTurn,angleVelocityInSteps*whichWayToTurn,false,odometrySubscriber,velocityPublisher);
                             end
                        end
                        firstTimeParallelFlag = false;
                    else
                        objectBlocking = true;
                    end
                end
            end
        %end
    end
end

function [obstacleInTheWay] = isObstacleInTheWay(distanceToObstacle, maxDistanceToObstacleMiddle, maxDistanceToObstacleSides, rightDistanceValues, leftDistanceValues, scan)
    obstacleInTheWay = (~isnan(distanceToObstacle) && distanceToObstacle < maxDistanceToObstacleMiddle) || mean(scan.Ranges(rightDistanceValues),'omitnan') < maxDistanceToObstacleSides || mean(scan.Ranges(leftDistanceValues),'omitnan') < maxDistanceToObstacleSides;
end
function [obstacleInTheWayWithFarSides] = isObstacleInTheWayWithFarSides(sensorCenterDistanceValues, leftDistanceValues, farRightDistanceValues, maxDistanceToObstacleMiddle, maxDistanceToObstacleSides, maxDistanceToObstacleFarSides, rightDistanceValues, farLeftDistanceValues)
    obstacleInTheWayWithFarSides = ~(mean(lidarScan.Ranges(sensorCenterDistanceValues),'omitnan') < maxDistanceToObstacleMiddle ||....
        mean(lidarScan.Ranges(rightDistanceValues),'omitnan') < maxDistanceToObstacleSides ||...
        mean(lidarScan.Ranges(leftDistanceValues),'omitnan') < maxDistanceToObstacleSides ||...
        mean(lidarScan.Ranges(farLeftDistanceValues),'omitnan')<maxDistanceToObstacleFarSides||...
        mean(lidarScan.Ranges(farRightDistanceValues),'omitnan')<maxDistanceToObstacleFarSides);
end