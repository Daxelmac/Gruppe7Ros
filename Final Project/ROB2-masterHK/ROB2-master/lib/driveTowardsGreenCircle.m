function [aoa] = driveTowardsGreenCircle(scansub, velpub, odomsub) 

% Calculate distance to drive in first step
scandata = receive(scansub);
cart = readCartesian(scandata);
idx = abs(cart(:,2)) == min(abs(cart(:,2)));
dist = cart(idx, 1);
x = cart(:,2); % x-pos
d = cart(:,1); % depth

% Select center measurement
c_idx = find(idx==1);
xcenter = x((c_idx-50):(c_idx+50));
dcenter = d((c_idx-50):(c_idx+50));

% xleft = x(100:200)
% dleft = d(100:200)
mdl = fitlm(xcenter,dcenter);
coef = mdl.Coefficients.Estimate;
angleEst = atand(coef(2));

if angleEst < 0
	aoa = 1;
elseif angleEst > 0
	aoa = 2;
end

% Perpendicular distance to wall
perpDist = cosd(angleEst)*dist;

% Distance to drive
driveDist = ((perpDist-1)/perpDist)*dist;

moveRelative(driveDist, 0.2, velpub, odomsub)