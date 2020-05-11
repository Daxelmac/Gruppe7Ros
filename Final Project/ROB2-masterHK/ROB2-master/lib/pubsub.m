function [velpub, odomsub, imsub, scansub] = pubsub()
% Subscribe to and Publish the ROS messages we use in the project.
%   Detailed explanation goes here

if ismember('/camera/rgb/image_raw',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_raw');
end

if ismember('/mobile_base/commands/velocity', rostopic('list'))
	velpub = rospublisher('/mobile_base/commands/velocity');
end

if ismember('/odom', rostopic('list'))
	odomsub = rossubscriber('/odom');
end

if ismember('/scan', rostopic('list'))
	scansub = rossubscriber('/scan');
end

end

