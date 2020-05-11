% establishConnection()
%% Init
close all
% point = rossubscriber('/camera/depth_registered/points');

% pointcloud = receive(point,3) %Point cloud acquisition
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
xlabel('x'), ylabel('y'), zlabel('z')

xyzc = xyz(~isnan(xyz(:,3)),:); % remove NaN

%% Remove data with negative y-koordinates


wall_i = xyzc(:,2) < 0;
wall_xyzc = xyzc(wall_i,:);

%% Segmented/clustered point cloud (kmeans)
K = 10;
[idx,C,sumd] = kmeans(wall_xyzc, K, 'Display', 'iter'); 
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
    scatter3(wall_xyzc(idx==i,1),wall_xyzc(idx==i,2), wall_xyzc(idx==i,3), '.')
    hold on
end
xlabel('x'), ylabel('y'), zlabel('z')

%% Plane fitting on single cluster
N= zeros(1,K);

for i = 1:K
    
    %M = 6; % cluster id
    %figure
    %scatter3(wall_xyzc(idx==i,1),wall_xyzc(idx==i,2), wall_xyzc(idx==i,3), '.')

    P = wall_xyzc(idx==i,:)';    % Plane fitting..
    N(1,i) = size(P,2); % number points 
end

%% Single segment point cloud 
% N_max = 0;
% [N_max, index] = max(N) 
% 
% figure
% scatter3(wall_xyzc(idx==index,1),wall_xyzc(idx==index,2), wall_xyzc(idx==index,3), '.')

%% Eigenvalue arrays

v2=zeros(3,3,K);
d2=zeros(3,3,K);

x0=zeros(K,3);
for i = 1:K
    
    P = wall_xyzc(idx==i,:)';    % Plane fitting..
    
    x0(i,:) = mean(P');
    
    % SIMILAR TO Plane_fitting_example_CORKE.m
    P0 = P - repmat(x0(i,:)', 1, N(i));
    w = ones(1,N(i)); % Weights - could be different..
    J = (repmat(w,3,1).*P0)*P0';
    J;  % show J

    [v2(:,:,i),d2(:,:,i)] = eig(J);  % eigen-vec/values
end

% Best eigen vectors
eigVec = zeros(3,K);
normVec = zeros(2,3,K);
for i = 1:K
	eigVec(:,i) = v2(:,1,i);
	normVec(:,:,i) = [x0(i,:); x0(i,:) - 0.5*eigVec(:,i)'];
end

% nvec = [x0 ; x0 + 0.5*n]
%Color segmentation after removing segments
figure
for i=1:K
	scatter3(wall_xyzc(idx==i,1),wall_xyzc(idx==i,2), wall_xyzc(idx==i,3), '.')
	
	hold on
	line(normVec(:,1,i), normVec(:,2,i), normVec(:,3,i))
end
xlabel('x'), ylabel('y'), zlabel('z')
hold off
%%
robotView = [0 0 0; 0 0 1];

wallLine = line(normVec(:,1,1), normVec(:,2,1), normVec(:,3,1));
robotLine = line(robotView(:,1,1), robotView(:,2,1), robotView(:,3,1));

Wvector = [wallLine.XData(2)-wallLine.XData(1), wallLine.YData(2)-wallLine.YData(1), wallLine.ZData(2)-wallLine.ZData(1)]
Rvector = [0, 0, 1]
Wvector2 = [wallLine.XData(1), wallLine.YData(1), wallLine.ZData(1)]

CosTheta = dot(Wvector,Rvector)/(norm(Wvector)*norm(Rvector));
ThetaInDegrees = 180-acosd(CosTheta)

vectorLength = norm(Wvector2)

figure()
scatter3(1,1,1)
line(normVec(:,1,1), normVec(:,2,1), normVec(:,3,1))
hold on
line(robotView(:,1,1), robotView(:,2,1), robotView(:,3,1))
hold off
%% Angle and length - in x- og z-plane
Wvectorxz = [wallLine.XData(2)-wallLine.XData(1), wallLine.ZData(2)-wallLine.ZData(1)]
Rvectorxz = [0, 1]
Wvector2xz = [wallLine.XData(1), wallLine.ZData(1)]

CosTheta = dot(Wvectorxz,Rvectorxz)/(norm(Wvectorxz)*norm(Rvectorxz));
ThetaInDegrees = acosd(CosTheta)

vectorLength = norm(Wvector2xz)

vinkelret = cos(ThetaInDegrees)*vectorLength
figure()
plot([0 Wvector2xz(1)], [0 Wvector2xz(2)])
hold on
plot([0 0], [0 vinkelret])


%% Laurits og Jonas' work
dwall = zeros(3,3,20);
dwall_i = zeros(1,20);
for i = 1:K
   if d(1,1,i) < 5
       dwall(:,:,i) = d(:,:,i)
	   dwall_i(1,i)=1;
       %scatter3(dwall(1,1,i), dwall(2,2,i), dwall(3,3,i), '.')
       %scatter3(dwall)
       %hold on
   end
end

%Color segmentation after removing segments
figure
for i=1:K
	if(dwall_i(1,i))
		scatter3(wall_xyzc(idx==i,1),wall_xyzc(idx==i,2), wall_xyzc(idx==i,3), '.')
		hold on
	end
end



%%
M = 20; % cluster id
figure(1)
scatter3(wall_xyzc(idx==M,1),wall_xyzc(idx==M,2), wall_xyzc(idx==M,3), '.')

P = wall_xyzc(idx==M,:)';    % Plane fitting..
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
scatter3(P(1,:), P(2,:), P(3,:), '.'), 
xlim([0 3]), ylim([0 3]), zlim([0 3])
xlabel('x'), ylabel('y'), zlabel('z')
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


Plength = norm(vectorP);
