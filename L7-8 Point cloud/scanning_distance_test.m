rosshutdown
setenv('ROS_MASTER_URI','http://192.168.7.34:11311')
setenv('ROS_IP','192.168.7.12')
rosinit('http://192.168.7.34:11311','NodeHost','192.168.7.12');

% Read scan continously
if ismember('/scan',rostopic('list'))
    scansub = rossubscriber('/scan');
    
    figure
    while(1)
        linescan = receive(scansub); %Receive message
        ranges = linescan.Ranges; % Extract scan
        angles = linescan.AngleMin:linescan.AngleIncrement:linescan.AngleMax;
        plot(angles, ranges)
        xlabel('Angle [rad]')
        ylabel('Distance [m]')
        %saveas(gcf,'linescan.png')
    end
end