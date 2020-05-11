establishConnection()
%% Init
close all
point = rossubscriber('/camera/depth_registered/points');

pointcloud = receive(point,3) %Point cloud acquisition
%% Load Data
clear
Vinkelret_1 = load('Vinkelret_1.mat') 
Vinkelret_2 = load('Vinkelret_2.mat') 
Vinkel_1 = load('Vinkel_1.mat') 
Vinkel_2 = load('Vinkel_2.mat') 

%Renaming data in workspace
Vinkelret_1 = Vinkelret_1.pointcloud
Vinkelret_2 = Vinkelret_2.pointcloud
Vinkel_1 = Vinkel_1.pointcloud
Vinkel_2 = Vinkel_2.pointcloud

%% Point cloud display
%read XYZ format
xyz = Vinkelret_1.readXYZ();
figure
scatter3(xyz(:,1),xyz(:,2), xyz(:,3))

xyzc = xyz(~isnan(xyz(:,3)),:); % remove NaN

%% Segmented/clustered point cloud (kmeans)
K = 20;
[idx,C,sumd] = kmeans(xyzc, K, 'Display', 'iter'); 
C = C'

%Dots
figure(1)
scatter3(xyz(:,1),xyz(:,2), xyz(:,3), '.')
hold on
for i=1:K
    plot3(C(1,i), C(2,i), C(3,i), 'r.')
end

%Color segmentation
figure
for i=1:K
    scatter3(xyzc(idx==i,1),xyzc(idx==i,2), xyzc(idx==i,3), '.')
    hold on
end

%% Plane fitting on single cluster
N= zeros(1,K)

for i = 1:K
    
    %M = 6; % cluster id
    %figure
    %scatter3(xyzc(idx==i,1),xyzc(idx==i,2), xyzc(idx==i,3), '.')

    P = xyzc(idx==i,:)';    % Plane fitting..
    N(1,i) = size(P,2); % number points 
end

%% Single segment point cloud 
N_max = 0;
[N_max, index] = max(N) 

figure
scatter3(xyzc(idx==index,1),xyzc(idx==index,2), xyzc(idx==index,3), '.')

%% Eigenvalue arrays

v=zeros(3,3,20);
d=zeros(3,3,20);

x0=zeros(K,3);
for i = 1:K
    
    P = xyzc(idx==i,:)';    % Plane fitting..
    
    x0(i,:) = mean(P');
    
    % SIMILAR TO Plane_fitting_example_CORKE.m
    P0 = P - repmat(x0(i,:)', 1, N(i));
    w = ones(1,N(i)); % Weights - could be different..
    J = (repmat(w,3,1).*P0)*P0';
    J;  % show J

    [v(:,:,i),d(:,:,i)] = eig(J);  % eigen-vec/values
end
%% Laurits og Jonas' work
dwall = zeros(3,3,20);
figure
for i = 1:K
   if d(1,1,i) < 5
       dwall(:,:,i) = d(:,:,i)
       %scatter3(dwall(1,1,i), dwall(2,2,i), dwall(3,3,i), '.')
       %scatter3(dwall)
       %hold on
   end
end

%%
M = 20; % cluster id
figure(1)
scatter3(xyzc(idx==M,1),xyzc(idx==M,2), xyzc(idx==M,3), '.')

P = xyzc(idx==M,:)';    % Plane fitting..
N = size(P,2); % number points

x0 = mean(P')

figure(2), scatter3(Vinkelret_1)

% SIMILAR TO Plane_fitting_example_CORKE.m
P0 = P - repmat(x0', 1, N);
w = ones(1,N); % Weights - could be different..
J = (repmat(w,3,1).*P0)*P0';
J  % show J

[v,d] = eig(J)  % eigen-vec/values

n = v(:,1)'% eigenvector corresponding to lowest eigenvalue..
figure(3)
scatter3(P(1,:), P(2,:), P(3,:), '.')
hold on
nvec = [x0 ; x0 + 0.5*n]; % normal vector for plotting (from mean to mean+normvec)
p0 = [0,0,0];
p1 = [0,0,2];
rnvec = [p0;p1];

lineM = line(nvec(:,1), nvec(:,2), nvec(:,3), 'Color', 'r')
hold on
lineR = line(rnvec(:,1), rnvec(:,2), rnvec(:,3), 'Color', 'g');
%%
vectorM = [p1-p0]; % Robot orientation
vectorP = [lineM.XData(2)-lineM.XData(1),lineM.YData(2)-lineM.YData(1),lineM.ZData(2)-lineM.ZData(1)];

CosTheta = dot(vectorM,vectorP)/(norm(vectorM)*norm(vectorP));
ThetaInDegrees = acosd(CosTheta);
