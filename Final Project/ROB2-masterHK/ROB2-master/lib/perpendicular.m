function [retur] = perpendicular()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%%
%Driving param
tolerance = 0.05;
ang_velo = 0.5;
laser = rossubscriber('/scan');
scan = receive(laser,3);
scan = receive(laser,3);
figure(1)
plot(scan);
avrRight = nanmean(scan.Ranges(1:320))  %Obs. data er invertet ift. plot
avrLeft = nanmean(scan.Ranges(321:640)) %Obs. data er invertet ift. plot

while abs(avrLeft-avrRight) > tolerance
    
   if avrLeft>avrRight
       rotateDegree(2,ang_velo,0);
   end
   if avrLeft<avrRight
       rotateDegree(-2,-ang_velo,0);
   end
   scan = receive(laser,3);
   avrRight = nanmean(scan.Ranges(1:320));
   avrLeft = nanmean(scan.Ranges(321:640));
end
figure(2)
plot(scan);
retur = nanmean(scan.Ranges(318:323));
end

