function [closeToGreenCircle, distance] = findDistance(scan, robotWidth)
    robotRadius = robotWidth/2;
    if(~isnan(scan.Ranges(319)))
        distance = scan.Ranges(319);
    else
        closeToGreenCircle = 0;
        distance = 0;
    end
    
    if(distance>0.4+robotRadius)
        closeToGreenCircle = 1;
    elseif(distance<0.4+robotRadius)
        closeToGreenCircle = 1;
    elseif(distance == 0.4+robotRadius)
        closeToGreenCircle = 1;
    end
end