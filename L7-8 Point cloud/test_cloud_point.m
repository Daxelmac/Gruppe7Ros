rosshutdown
setenv('ROS_MASTER_URI','http://192.168.7.34:11311')
setenv('ROS_IP','192.168.7.12')
rosinit('http://192.168.7.34:11311','NodeHost','192.168.7.12');

% Read scan continously
if ismember('/camera/depth/points',rostopic('list'))
    pointcloudsub = rossubscriber('/camera/depth/points');
    figure
    while(1)
        pc = receive(pointcloudsub); %Receive message
        scatter3(pc);
    end
    
end