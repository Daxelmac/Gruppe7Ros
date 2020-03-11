rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.100')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.100');

% Read scan continously
if ismember('/camera/depth_registered/points',rostopic('list'))
    pointcloudsub = rossubscriber('/camera/depth_registered/points');
    figure
    while(1)
        pc = receive(pointcloudsub); %Receive message
        scatter3(pc);
    end
    
end