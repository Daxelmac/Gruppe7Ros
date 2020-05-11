function [objectAvoided] = avoidObject(laserSub,odomSub,velPub,scan)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Defines:
stillInFrontFlag = false;
offsetcounter = 0;
midRange = 0.65;
firstMidRange = 0.8;
sideRange = 0.79;
wideSideRange = 0.45;
wideLeftValues = [620:640];
wideRightValues = [1:21];
parallelMoveSize = 0.3;
stepSize = 0.21;
step_ang_vel = 1;
step_lin_vel = 0.3;
objectAvoided = false;
midValues = [300:340];
rightValues = [90:113];
leftValues = [497:520];
	% Takes new scan
	%scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
    %figure(1);
    %plot(scan);
    ObDis = mean(scan.Ranges(midValues),'omitnan');
    if((~isnan(ObDis)  && ObDis<midRange) || mean(scan.Ranges(rightValues),'omitnan')<sideRange|| mean(scan.Ranges(leftValues),'omitnan')<sideRange)
		stopMotion(velPub);
        objectAvoided = true;
        stillInFrontFlag = true;
        firstTimeParallelFlag = true;  
        %disp('In If');
        pause(2);
        % Takes a new scan
        scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
        ObDis = mean(scan.Ranges(midValues),'omitnan');
        disp("ObDis1: ");
        disp(ObDis);
        %moveRelative(ObDis-(ObDis-midRange)+0.05,step_lin_vel,velPub,odomSub);
        % Takes a new scan
%         scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
%         ObDis = mean(scan.Ranges(midValues),'omitnan');
%         disp("ObDis2: ");
%         disp(ObDis);
        %if((~isnan(ObDis)  && ObDis<firstMidRange) || mean(scan.Ranges(rightValues),'omitnan')<sideRange|| mean(scan.Ranges(leftValues),'omitnan')<sideRange)
            while(offsetcounter ~= 0 || stillInFrontFlag == true)
                while(mean(scan.Ranges(midValues),'omitnan')<midRange || mean(scan.Ranges(rightValues),'omitnan')<sideRange || mean(scan.Ranges(leftValues),'omitnan')<sideRange)
                    %disp('In while');
                    %if (~isnan(ObDis) && ObDis<(midRange+0.1))
                        %disp('In if2');
                        ObDisLeft = mean(scan.Ranges(620:640),'omitnan'); 
                        ObDisRight = mean(scan.Ranges(1:21),'omitnan');
                        %figure(2); plot(scan);
                        % If NaNs are detected mean returns NaN.
                        % If NaNs to the right and to the left, prefer going left
                        ObDisLeft(isnan(ObDisLeft)) = 15;
                        ObDisRight(isnan(ObDisRight)) = 14;
                        if(ObDisLeft>ObDisRight)
                            turningFactor = 1;
                            offsetcounter = offsetcounter -1;
                            %disp(turningFactor);
                        else
                            turningFactor = -1;
                            offsetcounter = offsetcounter +1;
                            %disp(turningFactor);
                        end
                   % end
                    rotateDegree2(90*turningFactor,step_ang_vel*turningFactor,false,odomSub,velPub);
                    moveRelative(stepSize,step_lin_vel,velPub,odomSub);
                    rotateDegree2(-90*turningFactor,-step_ang_vel*turningFactor,false,odomSub,velPub);
                    % Takes a new scan
                    scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
                end
                stillInFrontFlag = false;

                % Now, maybe parallel with the object
                while(offsetcounter ~= 0 && stillInFrontFlag == false)
                   % disp('In while offsetCounter~=0');
                   % disp(offsetcounter);
                    % Takes a new scan
                    scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
                    if(~(mean(scan.Ranges(midValues),'omitnan')<midRange || mean(scan.Ranges(rightValues),'omitnan')<sideRange ||...
                            mean(scan.Ranges(leftValues),'omitnan')<sideRange||mean(scan.Ranges(wideLeftValues),'omitnan')<wideSideRange||...
                            mean(scan.Ranges(wideRightValues),'omitnan')<wideSideRange))
                        %disp('In move parallel');
                        stillInFrontFlag = false;
                        % Drives forward
                        moveRelative(parallelMoveSize,step_lin_vel,velPub,odomSub);
                        if(~firstTimeParallelFlag)    
                            rotateDegree2(-90*turningFactor,-step_ang_vel*turningFactor,false,odomSub,velPub);
                            % Takes a new scan
                             scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
                             if(mean(scan.Ranges(midValues),'omitnan')<midRange || mean(scan.Ranges(rightValues),'omitnan')<sideRange || mean(scan.Ranges(leftValues),'omitnan')<sideRange)
                                 %disp('Not behind');
                                 rotateDegree2(90*turningFactor,step_ang_vel*turningFactor,false,odomSub,velPub);
                             else
                                 %disp('maybe behind');
                                 while(offsetcounter ~= 0 && ~(mean(scan.Ranges(midValues),'omitnan')<midRange || mean(scan.Ranges(leftValues),'omitnan')<sideRange || mean(scan.Ranges(leftValues),'omitnan')<sideRange))
                                     moveRelative(stepSize,step_lin_vel,velPub,odomSub);
                                     % Takes a new scan
                                    scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
                                    offsetcounter = offsetcounter + turningFactor;
                                    %disp(offsetcounter);
                                 end
                                rotateDegree2(90*turningFactor,step_ang_vel*turningFactor,false,odomSub,velPub);
                             end
                        end
                        firstTimeParallelFlag = false;
                    else
                        stillInFrontFlag = true;
                    end
                end
            end
        %end
    end
end

