function [] = moveRelative(distance, vel, velpub, odomsub)
%Moves the turtlebot a given distance
%   Detailed explanation goes here

velmsg = rosmessage(velpub);
velmsg.Linear.X = vel;

% Set start and current position
odomdata = receive(odomsub);
Xstart = odomdata.Pose.Pose.Position.X;
Ystart = odomdata.Pose.Pose.Position.Y;

Xcurrent = Xstart;
Ycurrent = Ystart;

% Calculate moved distance
start = [Xstart, Ystart];
current = [Xcurrent, Ycurrent];
moveVec = norm(current-start);

% Adjust distance to observed overshoot.
distance = distance - 0.02;


% Drive until the desired distance is reached.
while moveVec < distance
	send(velpub, velmsg);
	odomdata = receive(odomsub);
	Xcurrent = odomdata.Pose.Pose.Position.X;
	Ycurrent = odomdata.Pose.Pose.Position.Y;
	start = [Xstart, Ystart];
	current = [Xcurrent, Ycurrent];
	moveVec = norm(current-start);

end
stopMotion(velpub);

end

