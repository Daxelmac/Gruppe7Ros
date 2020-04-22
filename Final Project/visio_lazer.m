rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.65:11311')
setenv('ROS_IP','192.168.1.56')
rosinit('http://192.168.1.193:11311','NodeHost','192.168.1.56');

robot = rospublisher('/mobile_base/commands/velocity') ;
%%
Tolerance=50;
SizeGoal = 200;

[CenterHorizontal, ObjectCenterHorizontal, MajorAxis] = picture()

while 1
    while or(ObjectCenterHorizontal >= CenterHorizontal + Tolerance...
           , ObjectCenterHorizontal <= CenterHorizontal - Tolerance)

        if ObjectCenterHorizontal > CenterHorizontal
           rotateDegree(3,0.3,0);

        end

        if ObjectCenterHorizontal < CenterHorizontal
           rotateDegree(-3,-0.3,0);
        end

        close all
        [CenterHorizontal, ObjectCenterHorizontal, MajorAxis] = picture()
    end
   
     while MajorAxis < SizeGoal    

            for i = 1:10
            move(0.4,0,robot) %move forward
            pause(0.1)

            end

            [CenterHorizontal, ObjectCenterHorizontal, MajorAxis] = picture()
            break
     end
     
end
%%
function [center,objectcenter,majoraxis] = picture()
close all

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
I = 2*imG - (imB+imR);

threshold = 150;
img_bin = zeros(size(I));
thres = I>threshold;
img_bin(thres)=255;

img_bin = img_bin > 0;

%figure, imshow(img_bin)

%Opening - Not necessary in simulation, due to no noise
SE = ones(5); %Structuring Element
img_bin_open = imopen(img_bin,SE); %Erosion

myStats = regionprops(img_bin_open, 'Area', 'Centroid', 'Circularity', 'MajoraxisLength'); % finding properties of connected components..

% Plot original image with center of brick
myCirc = myStats(1).Circularity;
myArea = myStats(1).Area;
myCenter = myStats(1).Centroid;
myMajorAxis = myStats(1).MajorAxisLength;

objectcenter = myCenter(1);
center = size(img,2)/2;
majoraxis = myMajorAxis

figure, imshow(img_bin_open)
hold on
plot(myCenter(1), myCenter(2), 'r.', 'MarkerSize', 30)
end

%%