%% Wheel Steering Angle Profile Generator 2

function [W_SA]=SP3(t,tstart,tstop,amp)
% Creates wheel steering angle input profile with. Full Sinusoidal input
% spanning front a specified start to stop time and with a prescribed peak
% amplitude.
%   - generates steering angle 
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
% K. Barreto
% Version 1/3/22


T = tstop-tstart;
b = (2*pi)/T;

x = length(t);
i_coast = find(t==tstart)-1;
i_decel = find(t==tstop)-1;
W_SA = zeros(1,x);

for i = 1:x
    if i <= i_coast
        W_SA(i) = 0;
    elseif (i>i_coast) && (i<=i_decel) 
        SA = -amp*(sin((b)*(t(i)-t(i_coast+1))));
        W_SA(i) = -SA;
    else
        W_SA(i)=0;
    end
end