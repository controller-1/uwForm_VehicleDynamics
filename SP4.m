%% Wheel Steering Angle Profile Generator 4

function [W_SA]=SP4(t,tstart,tstop,amp)
% Creates wheel steering angle input profile with smooth sinusoidal-like input
% spanning front a specified start to stop time and with a prescribed peak
% amplitude.
%
% Inputs:
%   t; time array
%   tstart; input start time (sec.)
%   tstop; input stop time (sec.)
%   amp; desired peak steering amplitude at the front wheels (deg)
%       - assumes parallel steering at both front wheels
%
% Outputs:
%   W_SA; wheel steering angle profile, 1xn array, (deg)
%
%
% Note: Not a true sinusoid, but rather two halves of a sinusoid with the 
% edges rounded off. Rounded edges to simulate a more realistic driver
% steering profile.
%
%
% K. Barreto
% MSME - University of Washington
% UW Formula
% 1/18/2022


T = tstop-tstart;                   % time range of when brakes are applied
b = (4*pi)/T;

x = length(t);
i_coast = find(t==tstart)-1;        % index in time array when start applying brakes
i_mid   = find(t==(tstart+T/2))-1;  % index at middle of time range
i_decel = find(t==tstop)-1;         % index at end of brake apply phase
W_SA = zeros(1,x);                  % initialize array to store output

for i = 1:x
    if i <= i_coast
        W_SA(i) = 0;
    elseif (i>i_coast) && (i<=i_mid) 
        SA = -amp*0.5*(sin((b)*(t(i)-t(i_coast+1))+pi/2))+0.5*amp;
        W_SA(i) = SA;
    elseif(i>i_mid) && (i<=i_decel) 
        SA = -amp*0.5*(sin((b)*(t(i)-t(i_mid+1))+pi/2))+0.5*amp;
        W_SA(i) = -SA;
    else
        W_SA(i)=0;
    end
end
% Smoother
windowsize = 100;
b = (1/windowsize)*ones(1,windowsize);
a = 1;
W_SA = filter(b,a,W_SA);
