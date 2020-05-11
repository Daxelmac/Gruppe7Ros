function [position] = getPosition(odomSub, startPositionInMeters)
    % Get turtlebot position 
    odomdata = receive(odomSub, 2); 
    pose = odomdata.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    % Set position from received data
    position = [pose.Position.X+startPositionInMeters(1), pose.Position.Y+startPositionInMeters(2), angles(1)];
end