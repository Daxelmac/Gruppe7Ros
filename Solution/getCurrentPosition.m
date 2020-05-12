function [currentPosition] = getCurrentPosition(odomSub, startPositionInMeters)
    % Inspiration from https://se.mathworks.com/help/ros/ug/communicate-with-the-turtlebot.html
    % Get the current position of the turtlebot 
    odomData = receive(odomSub, 2);
    position = odomData.Pose.Pose;
    quaternion = position.Orientation;
    angles = quat2eul([quaternion.W quaternion.X quaternion.Y quaternion.Z]);
    
    % Set position from received odometry data
    currentPosition = [position.Position.X + startPositionInMeters(1), position.Position.Y + startPositionInMeters(2), angles(1)];
end