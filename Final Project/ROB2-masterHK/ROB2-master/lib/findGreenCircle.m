function [center, objectcenter, majoraxis] = findGreenCircle(imsub)
%Find green circle in image
%   Detailed explanation goes here

close all;
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


%Opening - Not necessary in simulation, due to no noise
SE = ones(5); %Structuring Element
img_bin_open = imopen(img_bin,SE); %Erosion

myStats = regionprops(img_bin_open, 'Centroid', 'MajoraxisLength'); % finding properties of connected components..

% Plot original image with center of brick
if size(myStats, 1) == 0
	center = size(img,2)/2;
	objectcenter = 0;
	majoraxis = 0;
else
myCenter = myStats(1).Centroid;
myMajorAxis = myStats(1).MajorAxisLength;

objectcenter = myCenter(1);
center = size(img,2)/2;
majoraxis = myMajorAxis;


end

