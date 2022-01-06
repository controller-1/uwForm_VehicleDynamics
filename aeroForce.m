% This function returns aero force, due to either lift of drag (output
% force is respective of the input coefficient cD or cA).
%
% It accepts cD or cA as the 1st parameter, the frontal area as the
% 2nd parameter (typical normalized to 1 @ UWAL), and vehicle speed in
% MPS as the 3rd parameter.
%
% The coefficient and velocity inputs may be vectors, but if more than one 
% input is a vector, their sizes must match. If vectors are input, the 
% returned parameter will be the same size as the input vectors.
%
% Writen by Daniel Wageman
% Updated 141026
%
function Force = aeroForce(coefficient, frontarea, velocity)
% Environmental parameters
TempC       = 25;             % Outside temp in Deg C
Baro        = 101400;         % Atm Pressure in Pa
Rair        = 287.04;         % Ideal gas constant in J*kg^-1*K^-1
TempK       = 273 + TempC;  % Outside temp in Deg K

rho         = Baro / (Rair * TempK);   % Air Density in kg/m^3

% Calculate final force [ref: http://en.wikipedia.org/wiki/Drag_equation]
Force       = 0.5 * rho * (velocity ^ 2) * coefficient * frontarea;
end
% kg/m^3 * m^2/s^2 * m^2 = kg*m/s^2
%% CHANGELOG
% 141026 - Daniel Wageman
% Fully converted over to mps NOT kph