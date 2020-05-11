function [] = allignGreenCircle(Tolerance, angVel, imsub, odomsub, velpub)  

% Turn toward the green circle

[imCenter, circleCenter, ~] = findGreenCircle(imsub);

while or(circleCenter >= imCenter + Tolerance...
           , circleCenter <= imCenter - Tolerance)

        if circleCenter > imCenter
           rotateDegree2(-angVel, -1, 0, odomsub, velpub);
        end

        if circleCenter < imCenter
           rotateDegree2(angVel, 1, 0, odomsub, velpub);
        end

        close all
        [imCenter, circleCenter, ~] = findGreenCircle(imsub);
end

%hold on
%plot(imCenter, 200, 'rO')
%plot(circleCenter, 200, 'bO')
%hold off


