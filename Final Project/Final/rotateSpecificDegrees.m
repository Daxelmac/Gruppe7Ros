function [] = rotateSpecificDegrees(degrees, angularVelocity, odometrySubscriber, velocityPublisher)
angularToleranceHigh = 3;
angularToleranceLow = 0.3;

% Limit angular speed
angularSpeedLimit = 2;
if angularVelocity > angularSpeedLimit
	angularVelocity = angularSpeedLimit;
end
if angularVelocity < -angularSpeedLimit
	angularVelocity = -angularSpeedLimit;
end

% Get current angle and discard position
[~, ~, currentAngle] = getRobotPosition(odometrySubscriber);

% Calculate start and stop angle
startAngle = currentAngle;
finalAngle = startAngle + degrees;

% Keep the angle in degrees between -180 and 180
if finalAngle > 180
	angleOffSet = mod(finalAngle, 180);
	finalAngle = -180 + angleOffSet;
elseif finalAngle < -180
	angleOffSet = mod(finalAngle, -180);
	finalAngle = 180 + angleOffSet;
end

% Setup velocity message
velocityMessage = rosmessage(velocityPublisher);
velocityMessage.Angular.Z = angularVelocity;

while (currentAngle > (finalAngle + angularToleranceHigh) || currentAngle < (finalAngle - angularToleranceHigh))
	send(velocityPublisher, velocityMessage);
	[~, ~, currentAngle] = getRobotPosition(odometrySubscriber);
end
stopAllMovement(velocityPublisher);

% Get new current angle
[~, ~, currentAngle] = getRobotPosition(odometrySubscriber);

% Adjust direction to goal
if currentAngle > (finalAngle + angularToleranceLow) 
	velocityMessage.Angular.Z = -0.1;
elseif currentAngle < (finalAngle - angularToleranceLow)
	velocityMessage.Angular.Z = 0.1;
end


while (currentAngle > (finalAngle+angularToleranceLow) || currentAngle < (finalAngle - angularToleranceLow))
	send(velocityPublisher,velocityMessage);
	[~, ~, currentAngle] = getPose2(odometrySubscriber);
    
	% Set turn direction
	if currentAngle > (finalAngle + angularToleranceLow) 
		velocityMessage.Angular.Z = -0.1;
	elseif currentAngle < (finalAngle - angularToleranceLow)
		velocityMessage.Angular.Z = 0.1;
	end
end
stopAllMovement(velocityPublisher);
end

