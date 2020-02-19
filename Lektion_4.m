rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.100')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.100');
clc
clear

pause(5)

range = 2;

robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);

while(range > 1)
    % Grap image from camera
    if ismember('/camera/rgb/image_color/compressed',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_color/compressed');
    end

    if ismember('/camera/rgb/image_raw',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_raw');
    end

    imgraw = receive(imsub); % a serialised image
    img = readImage(imgraw); % decode image
    figure(1)
    imshow(img);
    
    m1g = rgb2gray(img);
    m1b = m1g < 130; % Image Segmenter APP
    m1bd = imclose(m1b, strel('disk', 10)); % fill holes in disc
    m1be = bwpropfilt(m1bd, 'Area', [1000 200000]); % remove small blobs..
    m1lab = bwlabel(m1be);
    m1prop = regionprops(m1lab, 'Area', 'Eccentricity'); % find features..
    m1bf = bwpropfilt(m1be, 'Eccentricity', 1, 'smallest');
    m1prop = regionprops(m1bf, 'Area', 'Eccentricity'); % find features..
    figure(2)
    imagesc(m1bf)
    
    if m1prop.Eccentricity < 0.6
       velocity = 0.05;
    end
    % Read scan continously
    if ismember('/scan',rostopic('list'))
        scansub = rossubscriber('/scan');
            linescan = receive(scansub); %Receive message
            ranges = linescan.Ranges; % Extract scan
            angles = linescan.AngleMin:linescan.AngleIncrement:linescan.AngleMax;
            %plot(angles, ranges)
            %xlabel('Angle [rad]')
            %ylabel('Distance [m]')
            %saveas(gcf,'linescan.png')
    end
    
    range = ranges(319);
    if isnan(range)
        range = 2;
    end
    
    if range <= 1
        velocity = 0;
    end

    velmsg.Linear.X = velocity;

    send(robot,velmsg);
end
