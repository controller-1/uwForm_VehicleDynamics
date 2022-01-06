%% Brake Profile 6

function [W_TB]=BP6(t,tstart,tstop,amp,bias)
% Creates custom brake profile that attempts to replicate a realistic
% profile. Only performs a single braking maneuver.
% 
%
% Inputs:
%   t; time array
%   tstart; input start time
%   tstop; input stop time
%   amp; desired brake torque at the front wheels (Not Total Torque)
%       - this method allows directly specifying front wheel brake torque
%       instead of calculating total torque for a specific brake bias in
%       order to get a specific front wheel brake torque output.
%   bias; ratio of total torque to front axle
%       i.e.: if amp=15 and bias=0.6
%           Torque front = 15Nm
%           Torque rear = (15*(1-bias))/bias = 10Nm
%
% Outputs:
%   W_TB; brake torque per wheel, 4x1 array
%
%
% K. Barreto
% Version 10/10/21

delBias = bias - 0.5;

x = length(t);
i_coast = find(t==tstart)-1;
i_decel = find(t==tstop)-1;
W_TB = zeros(4,x);
B = (2*pi)/(t(i_decel)-t(i_coast));
for i = 1:x
    if i <= i_coast
        W_TB(:,i ) = [0;0;0;0];
    elseif (i>i_coast) && (i<=i_decel) 
        famp = -amp*(bias-delBias)*(sin((B)*(t(i)-t(i_coast+1))+pi/2))+(bias-delBias)*amp;
        ramp = ((1-bias)/bias)*famp;
        if famp > 0 
            famp = 0;
        end
        if ramp > 0
            ramp = 0;
        end
        W_TB(:,i ) = [-famp;-famp;-ramp;-ramp];
    else
        W_TB(:,i )=[0;0;0;0];
    end
end