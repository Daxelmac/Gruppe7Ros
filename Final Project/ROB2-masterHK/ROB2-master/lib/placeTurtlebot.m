function [] = placeTurtlebot(xPose, yPose, yaw)
% Moves the turtlebot to the given coordinates in gazebo 

% Setup rospublisher
state = rospublisher('/gazebo/set_model_state');
stateMsg = rosmessage(state);

% Assign values
stateMsg.ModelName = 'mobile_base';
stateMsg.Pose.Position.X = xPose;
stateMsg.Pose.Position.Y = yPose;

eul = [yaw 0 0];
quat = eul2quat(eul); %wxyz
stateMsg.Pose.Orientation.W = quat(1);
stateMsg.Pose.Orientation.X = quat(2);
stateMsg.Pose.Orientation.Y = quat(3);
stateMsg.Pose.Orientation.Z = quat(4);

% Send message
send(state, stateMsg)
pause(2);
end