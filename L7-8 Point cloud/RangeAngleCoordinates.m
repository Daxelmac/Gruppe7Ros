function distanceAndAngleToClosestPoint = RangeAngleCoordinates()
    scansub = rossubscriber('/scan');
    linescan = receive(scansub); %Receive message

    scan = linescan;

    ranges = linescan.Ranges; % Extract scan
    angles = linescan.AngleMin:linescan.AngleIncrement:linescan.AngleMax;
    angles = angles';
    
    meas = [ranges, angles];
    % get data from scan
    if(isempty(scan))
        distanceAndAngleToClosestPoint = [];
    else
        for i = 1 : size(meas)
            if(~isnan(meas(i,1)))
                closestPoint = meas(i,:);
                break
            end
        end
        for i = 2 : size(meas)
            if(closestPoint(1,1) > meas(i,1) && ~isnan(meas(i,1)))
                closestPoint = meas(i,:);
            end
        end
        closestPoint(1,2) = rad2deg(closestPoint(1,2));
        distanceAndAngleToClosestPoint = closestPoint;
    end
end