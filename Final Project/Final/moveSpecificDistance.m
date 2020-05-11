function [] = moveSpecificDistance(distance, velocity, velocityPublisher, odometrySubscriber)
velocityMessage = rosmessage(velocityPublisher);
velocityMessage.Linear.X = velocity;

% Set start and current position
odometryData = receive(odometrySubscriber);
startPositionX = odometryData.Pose.Pose.Position.X;
startPositionY = odometryData.Pose.Pose.Position.Y;

currentPositionX = startPositionX;
currentPositionY = startPositionY;

% Calculate moved distance
startPosition = [startPositionX, startPositionY];
currentPosition = [currentPositionX, currentPositionY];
desiredDistanceVector = norm(currentPosition - startPosition);

% Adjust distance to observed overshoot.
distance = distance - 0.02;

% Drive until the desired distance is reached.
while desiredDistanceVector < distance
	send(velocityPublisher, velocityMessage);
	odometryData = receive(odometrySubscriber);
	currentPositionX = odometryData.Pose.Pose.Position.X;
	currentPositionY = odometryData.Pose.Pose.Position.Y;
	startPosition = [startPositionX, startPositionY];
	currentPosition = [currentPositionX, currentPositionY];
	desiredDistanceVector = norm(currentPosition - startPosition);
end
stopAllMovement(velocityPublisher);
end
