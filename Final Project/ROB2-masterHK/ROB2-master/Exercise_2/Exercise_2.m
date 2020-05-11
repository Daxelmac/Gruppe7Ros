rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.100')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.100');
rostopic list
%% Opgave 1
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
%% Opgave 2
Z = 200 % cm
X = 26 % cm
dr = 75 % n pixels
k =((X/Z)/dr)^(-1)
%% Opgave 3
Z = 100:300;
dr = k*(X./Z); 
figure(1);
plot(Z,dr); grid on;
figure(2)
plot(Z,1./Z); grid on;
%% Opgave 4
if ismember('/camera/rgb/image_color/compressed',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_color/compressed');
end
if ismember('/camera/rgb/image_raw',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_raw');
end
imgraw = receive(imsub); % a serialised image
% img = readImage(imgraw); % decode image
% figure(3)
% imshow(img(:,:,1));
% figure(4)
% imshow(img(:,:,2));
% figure(5)
% imshow(img(:,:,3));
% figure(6)
% imshow(2*img(:,:,2)-img(:,:,1)-img(:,:,3));
%%
test = img(:,:,2);
imgwrite(test,'imggreen');
%%
threshold = 50;

size(img(:,:,1))
img_bin = zeros(size(img(:,:,1)));
thres = find(img(:,:,2)>threshold);
img_bin(thres)=255;
figure(1)
imshow(img(:,:,1));
figure(2)
imshow(uint8(img_bin));