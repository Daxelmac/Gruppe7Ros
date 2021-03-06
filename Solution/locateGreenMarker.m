function [] = locateGreenMarker(cameraSub, laserSub, velMsg, velPub, robotWidth)
    searchForGreenMarker = true;
    while searchForGreenMarker
        % Set the searching motion of the turtlebot
        velMsg.Linear.X = 0;
        velMsg.Angular.Z = 0.5;
        
        % Capture new image
        scan = receive(laserSub);
        image = readImage(receive(cameraSub));
        
        % Remove red and blue colors
        imageRed = image(:,:,1); % Red channel
        imageGreen = image(:,:,2); % Green channel
        imageBlue = image(:,:,3); % Blue channel
        imageOnlyGreen = 2 * imageGreen - (imageBlue + imageRed);
        imageLogicalValues = logical(imageOnlyGreen);

        % Inspiration from week 3: "object detection example"
        % Remove noise from image
        imageNoNoise = bwpropfilt(imageLogicalValues, 'Area', [300 35000]); 
        % Remove high eccentricity 
        imageNoEccentricity = bwpropfilt(imageNoNoise, 'Eccentricity', 1, 'smallest');

        figure(16), imagesc(imageNoEccentricity)

        % Find features of image
        imageProperties = regionprops(imageNoEccentricity, 'Eccentricity', 'Centroid', 'Area', 'Circularity', 'BoundingBox');
        
        if ~isempty(imageProperties)
            for i = 1 : size(imageProperties, 1)
                % See if green marker is detected
                if(imageProperties.Eccentricity(1) < 0.8 && imageProperties.Circularity > 0.8)
                    % Green marker found
                    isGreenMarkerDetected = true;
                else
                    % Green marker wasn't a part of the properties
                    isGreenMarkerDetected = false;
                end
                center = imageProperties.Centroid;
                greenMarkerXCoord = int32(center(1, 1));
                if(~isnan(scan.Ranges(319))) % Picked as being center of the scanner
                    distanceToGreenMarker = scan.Ranges(319); % Picked as being center of the scanner
                else
                    closeToGreenCircle = false;
                    distanceToGreenMarker = 2; % Just large enough to make it try again
                end
                robotRadius = robotWidth / 2;
                wantedDistanceToGreenMarker = 0.4  + robotRadius; % 0.4 is the requirment from the assignment
                if(distanceToGreenMarker > wantedDistanceToGreenMarker)
                    closeToGreenCircle = false;
                elseif(distanceToGreenMarker <= wantedDistanceToGreenMarker)
                    closeToGreenCircle = true;
                end
                if (isGreenMarkerDetected || closeToGreenCircle)
                    % In here the turtlebot is trying to align and drive up
                    % to the green marker
                    if(greenMarkerXCoord < 320) 
                        velMsg.Angular.Z = 0.2; % Picked from trial and error, gains a smooth turning
                    elseif (greenMarkerXCoord > 320)
                        velMsg.Angular.Z = -0.2; % Picked from trial and error, gains a smooth turning
                    end
                    velMsg.Linear.X = 0.1; % Ensures slow movement towards marker
                    if(closeToGreenCircle)
                        return;
                    end
                end
            end
        end        
        velPub.send(velMsg);        
    end
end
