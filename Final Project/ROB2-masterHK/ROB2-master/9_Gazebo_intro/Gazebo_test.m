%%
establishConnection('1.164','1.144');
%%
a = readRGBimage();
imshow(a)
%%
close all
laser = rossubscriber('/scan');

scan = receive(laser,3)

figure
plot(scan);

%%
% Making objects for subscribing and publishing the topics
robot = rospublisher('/mobile_base/commands/velocity') ;
odom = rossubscriber('/odom');
resetOdometry = rospublisher('/mobile_base/commands/reset_odometry');
laser = rossubscriber('/scan');
scan = receive(laser,3)
i = 0
% Reset odometry
reset_msg = rosmessage(resetOdometry);
send(resetOdometry,reset_msg);
pause(2)

% Creating waypoints for the two rectangles, that is presenting the path
wp_little_rectangle1 = [[0;0] [1;0] [1;2] [0;2]]';
wp_little_rectangle2 = [[0;2] [-1;2] [-1;0] [0;0]]';

% Determined the start and goal for path 1 and 2
start1 = [0;0];
goal1 = [0;2];
start2 = goal1;
goal2 = start1;
goalRadius = 0.1;   % How tolerente may the robots position be to goal to 
                    % accomplish the "specific" task

% Round counter and desired round number
round_num = 1;
current_round = 1;

% Get initial pose
pose = getPose(odom);

% Creating array for logging the position of the robots movement - from
% its perspective 
logarray = zeros(100,2);
index = 1;

% Making the 
while (current_round<(round_num+1))
distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-goal1)
controller1 = controllerPurePursuit('DesiredLinearVelocity',0.3,'LookaheadDistance',0.3,'MaxAngularVelocity',1,'Waypoints',wp_little_rectangle1)
controller2 = controllerPurePursuit('DesiredLinearVelocity',0.3,'LookaheadDistance',0.3,'MaxAngularVelocity',1,'Waypoints',wp_little_rectangle2)

% While loop for first half of rectangle
    while (distanceToGoal1>goalRadius)
        % Position update
        pose = getPose(odom);
        % Convert orientation
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        % Calculation distance to goal
        distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-goal1);
        % Asking controller 1 for linear velocity and angular velocity
        [vel,ang_vel] = controller1([pose.Position.X, pose.Position.Y, angles(1)]);
        % Tells the robot to move
        move(vel,ang_vel,robot);
        % Logging data and incrementing index
        logarray(index,1) = pose.Position.X;
        logarray(index,2) = pose.Position.Y;
        index = index + 1;
        i = i+1;
        if i == 100
            scan = receive(laser,3);
            plot(scan);
            i = 0;
        end
    end
    
distanceToGoal2 = norm([pose.Position.X;pose.Position.Y]-goal2);

% While loop for second half of rectangle
    while (distanceToGoal2>goalRadius)
        % Position update
        pose = getPose(odom);
        % Convert orientation
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        % Calculation distance to goal
        distanceToGoal2 = norm([pose.Position.X;pose.Position.Y]-goal2);
        % Asking controller 1 for linear velocity and angular velocity
        [vel,ang_vel] = controller2([pose.Position.X, pose.Position.Y, angles(1)]);
        % Tells the robot to move
        move(vel,ang_vel,robot);
        % Logging data and incrementing index
        logarray(index,1) = pose.Position.X;
        logarray(index,2) = pose.Position.Y;
        index = index + 1;
        i = i+1;
        if i == 100
            scan = receive(laser,3);
            plot(scan);
            i = 0;
        end
    end
% Incrementing current round
current_round = current_round + 1;
end
%Stop robot, when near to final goal
move(0,0,robot);