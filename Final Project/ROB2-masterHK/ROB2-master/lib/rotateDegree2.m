function [] = rotateDegree2(degrees, ang_velocity, print_true, odomsub, velpub)
%rotateDegree Rotates the turtlebot 'degrees' positive meaning 
% clockwise with angular velocity 'ang_velocity'.
% Prints angles if 'print_true' == true


tolerance1 = 3;
tolerance2 = 0.3;


% Limit angular speed
w_limit = 2;
if ang_velocity > w_limit
	ang_velocity = w_limit;
end
if ang_velocity < -w_limit
	ang_velocity = -w_limit;
end



% Get current angle
[~, ~, current_degree] = getPose2(odomsub);

% Calculate start and stop angle
start_degree = current_degree;
final_degree = start_degree + degrees;

% Keep degrees between -180 and 180
if final_degree > 180
	offset = mod(final_degree, 180);
	final_degree = -180 + offset;
elseif final_degree < -180
	offset = mod(final_degree, -180);
	final_degree = 180 + offset;
end


% Setup velocity message
velmsg = rosmessage(velpub);
velmsg.Angular.Z = ang_velocity;         % Angular velocity (rad/s)


while (current_degree > (final_degree+tolerance1) || current_degree < (final_degree-tolerance1))
	send(velpub,velmsg);
	[~, ~, current_degree] = getPose2(odomsub);

	if (print_true)
		X = sprintf('Start: %f, Destination: %f, Current: %f', start_degree, final_degree, current_degree);
		disp(X);
	end
end
stopMotion(velpub);
fprintf('\n');



%% Precision step

% Get angle
[~, ~, current_degree] = getPose2(odomsub);

% Adjust direction to goal
if current_degree > (final_degree + tolerance2) 
	velmsg.Angular.Z = -0.1;%-ang_velocity/10;
elseif current_degree < (final_degree - tolerance2)
	velmsg.Angular.Z = 0.1;%ang_velocity/10;
end


while (current_degree > (final_degree+tolerance2) || current_degree < (final_degree-tolerance2))

	send(velpub,velmsg);
	[~, ~, current_degree] = getPose2(odomsub);

	if (print_true)
		X = sprintf('Start: %f, Destination: %f, Current: %f, Angle Vel: %f',start_degree,final_degree,current_degree, velmsg.Angular.Z);
		disp(X);
	end
	% Set turn direction
	if current_degree > (final_degree + tolerance2) 
		velmsg.Angular.Z = -0.1;%-ang_velocity/10;
	elseif current_degree < (final_degree - tolerance2)
		velmsg.Angular.Z = 0.1;%ang_velocity/10;
	end
end
stopMotion(velpub);

end

