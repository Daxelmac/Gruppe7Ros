function [] = rotateToDeg(angle, print_true, odomsub, velpub)
%Rotate the turtlebot to a specific angle (in degrees)
%   Detailed explanation goes here

tolerance = 3;



% Get current angle
[~, ~, current_degree] = getPose2(odomsub);
start_angle = current_degree;
final_angle = angle;

% Keep degrees between -180 and 180
if final_angle > 180
	final_angle = -180;
elseif final_angle < -180
	final_angle = 180;
end


% Adjust turn direction to goal
ang_velocity = 1;
if current_degree > final_angle
	ang_velocity = -1;
elseif current_degree < final_angle
	ang_velocity = 1;
end
% Invert turn direction, if crossing -180/180 is shorter
if abs(current_degree - final_angle) > 180
	ang_velocity = ang_velocity * (-1);
end


% Setup velocity message
velmsg = rosmessage(velpub);
velmsg.Angular.Z = ang_velocity;         % Angular velocity (rad/s)


while (current_degree > (final_angle+tolerance) || current_degree < (final_angle-tolerance))
	send(velpub,velmsg);
	[~, ~, current_degree] = getPose2(odomsub);
	
	% Print position
	if (print_true)
		X = sprintf('Start: %f, Destination: %f, Current: %f', ...
			start_angle, final_angle, current_degree);
		disp(X);
	end
	
	% Adjust turn direction to goal
	if current_degree > (final_angle + tolerance) 
		velmsg.Angular.Z = -1;
	elseif current_degree < (final_angle - tolerance)
		velmsg.Angular.Z = 1;
	end
	if abs(current_degree - final_angle) > 180
		velmsg.Angular.Z = velmsg.Angular.Z * (-1);
	end

end
stopMotion(velpub);

end

