rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.1')
rosinit('http://192.168.1.200:11311');

velocity = 0.1;

robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);

velmsg.Linear.X = velocity;

velmsg.Angular.Z = 0.6;

send(robot,velmsg);
