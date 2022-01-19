function [Fx,Fy] = TireForces_COG(Fl,Fc,dw,deltaMax)
% Converts the longitudinal/lateral tire forces of all 4 wheels from 
% a wheel reference frame in a 2 track system to the chassis undercarriage
% x-axis and y-axis force components.
% 
% Inputs:
%   Fl; longitudinal tire force, 4x1 array, [N]
%   Fc; lateral tire force, 4x1 array, [N]
%   dw; wheel steering angle [deg]
%   deltaMax; maximum wheel angle possible[deg]
% 
% Outputs:
%   Fx; x-axis tire forces, 4x1 array, [N]
%   Fy; y-axis tire forces, 4x1 array, [N]
%
% K.Barreto
% MSME - University of Washington
% UW Formula
% 1/18/2022
% 
% Assumptions of this model:
% - equal steering angle for both front wheels (2WS)

if dw > deltaMax
    msg = 'Error: Input steering angle greater than maximum allowed';
    error(msg)
end
deltarad = deg2rad(dw);
Fx_fl = Fl(1,1)*cos(deltarad) - Fc(1,1)*sin(deltarad);
Fx_fr = Fl(2,1)*cos(deltarad) - Fc(2,1)*sin(deltarad);
Fy_fl = Fl(1,1)*sin(deltarad) + Fc(1,1)*cos(deltarad);
Fy_fr = Fl(2,1)*sin(deltarad) + Fc(2,1)*cos(deltarad);
Fx_rl = Fl(3,1);
Fx_rr = Fl(4,1);
Fy_rl = Fc(3,1);
Fy_rr = Fc(4,1);
Fx = [Fx_fl;Fx_fr;Fx_rl;Fx_rr];
Fy = [Fy_fl;Fy_fr;Fy_rl;Fy_rr];
end