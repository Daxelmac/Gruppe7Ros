rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.100')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.100');
rostopic list
robot = rospublisher('/mobile_base/commands/velocity') ;
%%
velmsg = rosmessage(robot);
velocity = 0.2;
velmsg.Linear.X = velocity;
send(robot,velmsg);
%%
velmsg.Angular.Z = 0.6;	% Angular velocity (rad/s)
velmsg.Linear.X = 0; % Linear velocity (m/s), +forward,-reverse
%%
odom = rossubscriber('/odom');
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;
rz = pose.Orientation.Z;
%%
while true
    send(robot,velmsg);
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    x = pose.Position.X;
end
%%
velmsg.Angular.Z = 0.0;	% Angular velocity (rad/s)
velmsg.Linear.X = 0.2; % Linear velocity (m/s), +forward,-reverse
odom = rossubscriber('/odom');
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
start_x = pose.Position.X;
start_y = pose.Position.Y;
start_rz = pose.Orientation.Z;
while (x < start_x+0.1)
    send(robot,velmsg);
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;
    [x,y]
end 
velmsg.Angular.Z = 0.6;	% Angular velocity (rad/s)
velmsg.Linear.X = 0; % Linear velocity (m/s), +forward,-reverse
start_rz = pose.Orientation.Z;
while (rz < 2)
    send(robot,velmsg);
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    rz = pose.Orientation.Z;
    [rz]
end