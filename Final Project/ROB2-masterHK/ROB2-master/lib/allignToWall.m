function [] = allignToWall(scansub, odomsub, velpub) 

% Calculate parallel distance to move
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

mdl = fitlm(xcenter,dcenter);
coef = mdl.Coefficients.Estimate;
angleEst = atand(coef(2));

% Perpendicular distance to wall
perpDist = cosd(angleEst)*dist;

parallel = dist * sind(-angleEst);

fprintf('Angle to wall: %6.4f\n', angleEst);
fprintf('Perpendicular distance: %6.4f\n', perpDist);
fprintf('Parallel distance: %6.4f\n', parallel);

if parallel > 0 %aoa == 1
	angleBack = 90 -(-angleEst);
	rotateDegree2(-angleBack, 1, 0, odomsub, velpub)
	moveRelative(parallel-0.08, 0.1, velpub, odomsub)
	rotateDegree2(90, 1, 0, odomsub, velpub)
elseif parallel < 0 %aoa == 2
	angleBack2 = 90 -(angleEst);
	rotateDegree2(angleBack2, 1, 0, odomsub, velpub)
	moveRelative((-parallel)-0.08, 0.1, velpub, odomsub)
	rotateDegree2(-90, 1, 0, odomsub, velpub)
end

% dist - x = 0.4
scandata = receive(scansub);
cart = readCartesian(scandata);
idx = abs(cart(:,2)) == min(abs(cart(:,2)));
dist = cart(idx, 1);

driveDist = dist - 0.66;
moveRelative(driveDist, 0.1, velpub, odomsub)