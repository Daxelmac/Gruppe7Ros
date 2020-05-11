rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.100')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.100');
rostopic list

%%
Tolerance=30;

[CenterHorizontal, ObjectCenterHorizontal, MajorAxis] = picture()


while or(ObjectCenterHorizontal >= CenterHorizontal + Tolerance...
        , ObjectCenterHorizontal <= CenterHorizontal - Tolerance)
    
    if ObjectCenterHorizontal > CenterHorizontal
       rotateDegree(3,0.4);
       
    end
    
    if ObjectCenterHorizontal < CenterHorizontal
       rotateDegree(-3,-0.4);
    end
    
    close all
    [CenterHorizontal, ObjectCenterHorizontal, MajorAxis] = picture()
    
end

SizeGoal = 100

while MajorAxis < SizeGoal
    
    %move forward
    SizeGoal = 101;
   
end


%%
odom = rossubscriber('/odom');
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
x = pose.Position.X;

velmsg.Linear.X = 0.2; % Linear velocity (m/s), +forward,-reverse
odom = rossubscriber('/odom');
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
start_x = pose.Position.X;

while (x < start_x+0.4)
    send(robot,velmsg);
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    x = pose.Position.X;
end 
    
%% Image Feed
function [center,objectcenter,majoraxis] = picture()

if ismember('/camera/rgb/image_color/compressed',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_color/compressed');
end

if ismember('/camera/rgb/image_raw',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_raw');
end

imgraw = receive(imsub); % a serialised image
img = readImage(imgraw); % decode image

imR = img(:,:,1); % red channel
imG = img(:,:,2); % green channel
imB = img(:,:,3); % blue channel
I = 2*imR - (imG + imB);

threshold = 150;
img_bin = zeros(size(I));
thres = find(I>threshold);
img_bin(thres)=255;

img_bin = img_bin > 0;

%Opening
SE = ones(5); %Structuring Element

img_bin_erosion = imerode(img_bin,SE); %Erosion
img_bin_opened = imdilate(img_bin_erosion, SE); %Dilation

myStats = regionprops(img_bin_opened, 'Area', 'Centroid', 'Circularity', 'MajoraxisLength'); % finding properties of connected components..

% Plot original image with center of brick
myCirc = myStats(1).Circularity;
myArea = myStats(1).Area;
myCenter = myStats(1).Centroid;
myMajorAxis = myStats(1).MajorAxisLength;

objectcenter = myCenter(1);
center = size(img,2)/2;
majoraxis = myMajorAxis

figure, imshow(img_bin_opened)

end



