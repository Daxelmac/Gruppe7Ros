function [] = spinToLocateGreenCircle(steps, imsub, odomsub, velpub)  

for i = 1:steps
	[~, circleCenter, ~] = findGreenCircle(imsub);
	if circleCenter == 0
		rotateDegree2(360/steps, 1, 0, odomsub, velpub);
	else
		break
	end
end
