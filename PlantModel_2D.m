function [vcog,acog,beta,Yaw,XY,VXY,PSI]=PlantModel_2D(Fx,Fy,x0,m,Iveh,a,b,c,ts,DragFX)
% Performs simulation of nonlinear 2D chassis model
% Inputs:
%   Fx; longitudinal forces, 1x4 array, [N]
%   Fy; lateral forces, 1x4 array, [N]
%   x0; initial conditions, units: [m],[m/s],[m/s^2],[rad], [rad/s],[rad/s^2]
%       
%       x0 = [vx0; vy0; psi0; psidot0; X0; Y0]
%       
%       x, y; vehicle coordinates w.r.t initial vehicle ref. frame
%       vx,vy; velocity components in vehicle ref. frame 
%       ax,ay; acceleration components in veh. ref. frame
%       psi, psidot; angular position/velocity around vehicle COG
%       X,Y; vehicle coordinates in earth fixed ref. frame
%       PSI; angular position of vehicle w.r.t. earth fixed ref. frame
%   m; vehicle mass [kg]
%   Iveh; vehicle moment of inertia
%   a,b,c; length from COG to front/rear axle, wheel base [m]
%   ts; simulation time [s]
%   DragFX; aero drag effects on longitudinal axis of undercarriage [N]
% 
% Outputs: 
%   vcog; final vehicle velocity components in vehicle frame, 2x1 array
%       vcog = [vx; vy];
%   acog; final vehicle accel. components in vehicle frame, 2x1 array
%       acog = [ax; ay];
%   beta; vehicle side slip angle 
%   Yaw; vehicle yaw quantities 
%       Yaw = [position; ang. velocity; ang. accel.];
%   XY; vehicle coordinates w.r.t. earth fixed ref. frame  
%       XY = [X; Y];
%   VXY; vehicle velocity components w.r.t. earth fixed ref. frame
%       VXY = [VX; VY];
%   PSI; vehicle yaw quantities w.r.t. earth fixed ref. frame
%       PSI = [position; ang. velocity];
%
%
% K.Barreto
% MSME - University of Washington
% UW Formula
% Version: 1/18/2022

%###################################################################
%######################  Chassis Model   ###########################
%###################################################################

%       x0 = [vx0; vy0; psi0; psidot0; X0; Y0]
vx0             = x0(1,1);
vy0             = x0(2,1);
psi0            = x0(3,1);
psidot0         = x0(4,1);
X0              = x0(5,1);
Y0              = x0(6,1);

A = [(1/m)*(m*vy0*psidot0 + Fx(1) + Fx(2) + Fx(3) + Fx(4) - DragFX);
    (1/m)*(-m*vx0*psidot0 + Fy(1) + Fy(2) + Fy(3) + Fy(4));
    (1/Iveh)*(a*(Fy(1)+Fy(2))-b*(Fy(3)+Fy(4))+c*(-Fx(1)+Fx(2)-Fx(3)+Fx(4)))];
ax              = A(1,1);
ay              = A(2,1);
vx              = vx0 + ax*ts;
vy              = vy0 + ay*ts;
x               = vx0*ts + (1/2)*ax*ts^2;
y               = vy0*ts + (1/2)*ay*ts^2;
psiDdot         = A(3,1);
psidot          = psidot0 + psiDdot*ts;
psi             = psi0 + psidot0*ts + (1/2)*psiDdot*ts^2;
acog            = [ax;ay];
vcog            = [vx;vy];
Yaw             = [psi;psidot;psiDdot];
PSI             = [psi;psidot];
beta            = atan(vy/vx);
VX              = vx*cos(psi) - vy*sin(psi);
VY              = vx*sin(psi) + vy*cos(psi);
X               = x*cos(psi) - y*sin(psi) + X0;
Y               = x*sin(psi) + y*cos(psi) + Y0;
VXY             = [VX;VY];
XY              = [X;Y];
end