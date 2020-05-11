rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.100')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.100');
rostopic list
%% Scan
close all
laser = rossubscriber('/scan');

scan = receive(laser,3)

figure
plot(scan);

%% 20 second scan
tic;
while toc < 20
  scan = receive(laser,3);
  plot(scan);
end

%% Image Feed
close all
clear 

if ismember('/camera/rgb/image_color/compressed',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_color/compressed');
end
if ismember('/camera/rgb/image_raw',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_raw');
end
imgraw = receive(imsub); % a serialised image
img = readImage(imgraw); % decode image
figure
imshow(img);

imR = img(:,:,1); % red channel
%figure, imshow(imR)

imG = img(:,:,2); % green channel
%figure, imshow(imG)

imB = img(:,:,3); % blue channel
%figure, imshow(imB)

%I = 0.1*imR > (1.0*imG + 1.0*imB);
I = 2*imR - (imG + imB);

threshold = 150;
size(I)
img_bin = zeros(size(I));
thres = find(I>threshold);
img_bin(thres)=255;
figure(1)
imshow(img_bin);

%%
%Opening
SE = ones(5); %Structuring Element

img_bin_eroded = imerode(img_bin,SE); %Erosion
img_bin_dilated = imdilate(img_bin_eroded, SE); %Diolation

figure, imshow(img_bin_eroded)
figure, imshow(img_bin_dilated)

myStats = regionprops(img_bin_dilated, 'Area', 'Centroid', 'Circularity') % finding properties of connected

% Plot original image with center of brick
myCirc = myStats(1).Circularity
myArea = myStats(1).Area
myCenter = myStats(1).Centroid

ImageCenter = size(img,2)/2




