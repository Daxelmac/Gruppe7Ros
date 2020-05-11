map = binaryOccupancyMap(10,10,10);

x = [1.2; 2.3; 3.4; 4.5; 5.6];
y = [5.0; 4.0; 3.0; 2.0; 1.0];

setOccupancy(map, [x y], ones(5,1))
figure
show(map)

%%
map = zeros(400,400); % Each cell equals 1*1 cm
map(101:1:300,101:1:300) = ones(200,200);
figure

start = [0.01, 0.01];
goal = [2, 3];

imagesc(map)
hold on, plot(start(1), start(2), 'r*'), text(start(1), start(2), 'START')
hold on, plot(goal(1), goal(2), 'ro'), text(goal(1), goal(2), 'GOAL')
%%
bug = Bug2(map)
bug.goal = goal;
bug.verbose = 0;
%bug.query(start, goal, 'animate');
wp = bug.query(start, goal);
wp = wp./100;
wp = wp(1:16:size(wp,1),:);

%%hold on, plot(start(1), start(2), 'r*'), text(start(1), start(2), 'START')
%%hold on, plot(goal(1), goal(2), 'ro'), text(goal(1), goal(2), 'GOAL')
%%
establishConnection()
%%
% Reset odometry
robot = rospublisher('/mobile_base/commands/velocity') ;
odom = rossubscriber('/odom');
resetOdometry = rospublisher('/mobile_base/commands/reset_odometry');
reset_msg = rosmessage(resetOdometry);
send(resetOdometry,reset_msg);

pause(2)

%%wp_little_rectangle1 = [[0.5;0] [1;0] [1;1] [0.5;1]]'
%%wp_little_rectangle2 = [[0.5;1] [0;1] [0;0] [0.5;0]]'
wp_little_rectangle1 = [[0;0] [1;0] [1;2] [0;2]]';
wp_little_rectangle2 = [[0;2] [-1;2] [-1;0] [0;0]]';

start1 = [0;0];
goal1 = [0;2];
start2 = goal1;
goal2 = start1;
goalRadius = 0.1;

pose = getPose(odom);


counter = 1;
logarray = zeros(100,2);
current_round = 1;
while (current_round<11)
distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-goal1)
controller1 = controllerPurePursuit('DesiredLinearVelocity',0.3,'LookaheadDistance',0.3,'MaxAngularVelocity',4,'Waypoints',wp_little_rectangle1)
controller2 = controllerPurePursuit('DesiredLinearVelocity',0.3,'LookaheadDistance',0.3,'MaxAngularVelocity',4,'Waypoints',wp_little_rectangle2)
while (distanceToGoal1>goalRadius)
    pose = getPose(odom)
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    
    distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-goal1);
    
    [vel,ang_vel] = controller1([pose.Position.X, pose.Position.Y, angles(1)]);
    move(vel,ang_vel,robot);
    logarray(counter,1) = pose.Position.X;
    logarray(counter,2) = pose.Position.Y;
    counter = counter + 1;
end

distanceToGoal2 = norm([pose.Position.X;pose.Position.Y]-goal2);
while (distanceToGoal2>goalRadius)
    pose = getPose(odom);
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    
    distanceToGoal2 = norm([pose.Position.X;pose.Position.Y]-goal2);
    
    [vel,ang_vel] = controller2([pose.Position.X, pose.Position.Y, angles(1)]);
    move(vel,ang_vel,robot);
    logarray(counter,1) = pose.Position.X;
    logarray(counter,2) = pose.Position.Y;
    counter = counter + 1;
end
current_round = current_round + 1;
end
move(0,0,robot);


%%
figure(1)
plot(logarray(:,1),logarray(:,2))
hold on
plot(wp_little_rectangle1(:,1),wp_little_rectangle1(:,2), 'r')
plot(wp_little_rectangle2(:,1),wp_little_rectangle2(:,2), 'r')
hold off
%%plot(wp_rectangle(:,1),wp_rectangle(:,2));

%%
logarrayScaled = logarray*100;
figure(1)
hold on
imagesc(map)
hold on, plot(start(1), start(2), 'r*'), text(start(1), start(2), 'START')
hold on, plot(goal(1), goal(2), 'ro'), text(goal(1), goal(2), 'GOAL')