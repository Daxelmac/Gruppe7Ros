function [] = rotateDegree(degrees,ang_velocity,print_true)
%rotateDegree Rotates the turtlebot 'degrees' positive meaning 
% clockwise with angular velocity 'ang_velocity'.
% Prints angles if 'print_true' == true
tolerance = 1;
robot = rospublisher('/mobile_base/commands/velocity') ;
odom = rossubscriber('/odom');
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
start_degree = conv2Degree(pose.Orientation.Z);
final_degree = start_degree + degrees
if final_degree<0
   final_degree = 360+final_degree;
end

velmsg = rosmessage(robot);
velmsg.Angular.Z = -ang_velocity;         % Angular velocity (rad/s)
current_degree = start_degree

while (current_degree > (final_degree+tolerance) || current_degree < (final_degree-tolerance))
send(robot,velmsg);
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
current_degree = conv2Degree(pose.Orientation.Z);
if (print_true)
    X = sprintf('Start: %f, Destination: %f, Current: %f',start_degree,final_degree,current_degree);
    disp(X);
end
end
stopMotion(robot);


end

