function distanceAndAngleToClosestPoint = RangeAngleCoordinates()
    scansub = rossubscriber('/scan');
    linescan = receive(scansub); %Receive message

    scan = linescan;

    % get data from scan
    cart = readCartesian(scan);
    if(isempty(cart))
        distanceAndAngleToClosestPoint = [];
    else
        closestPoint = cart(1,:);
        for i = 2 : size(cart)
            if(closestPoint(1,1) > cart(i,1))
                closestPoint = cart(i,:);
            end
        end
        closestPoint(1,2) = rad2deg(closestPoint(1,2));
        distanceAndAngleToClosestPoint = closestPoint;
    end
end