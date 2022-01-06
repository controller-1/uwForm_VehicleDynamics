function [vcog,acog,beta,Yaw,XY,VXY,PSI]=PlantModel_2D_TEST(Fx,Fy,x0,m,Iveh,a,b,c,ts)
% Performs simulation of nonlinear 2D chassis model
% Inputs:
%   Fx; longitudinal tire forces, 1x4 array, [N]
%   Fy; lateral tire forces, 1x4 array, [N]
%   x0; initial conditions, units: [m],[m/s],[m/s^2],[rad], [rad/s],[rad/s^2]
%       
%       x0 = [x0; vx0; y0; vy0; psi0; psidot0; X0; Y0; PSI0]
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
% Version: 10/5/2021

%###################################################################
%######################  Chassis Model   ###########################
%###################################################################
%     function dx_dt = sys(t,x)
%     % x = [vx,ax,vy,ay,psiCOG,psiCOGdot,X,vX,Y,vY,PSI,PSIdot]
%     dx_dt = [x(1);
%         (1/m)*(m*x(3)*x(5)+Fx(1)+Fx(2)+Fx(3)+Fx(4));
%         x(3);
%         (1/m)*(-m*x(1)*x(5)+Fy(1)+Fy(2)+Fy(3)+Fy(4));
%         x(5);
%         (1/Iveh)*(a*(Fy(1)+Fy(2))-b*(Fy(3)+Fy(4))+c*(-Fx(1)+Fx(2)-Fx(3)+Fx(4)));
%         x(8)*t;
%         x(1)*cos(x(11))-x(3)*sin(x(11));
%         x(10)*t;
%         x(1)*sin(x(11))+x(3)*cos(x(11));
%         x(12)*t;
%         x(5)];
%     %FUTURE WORK: Add drag force and wind effects(speed and direction)
%     end
% [~,x] = ode45(@sys,[0, ts],x0);   %solve ODE, use default error size
% vx      = x(1,end);
% vy      = x(3,end);
% vcog    = [vx;vy];
% ax      = x(2,end);
% ay      = x(4,end);
% acog    = [ax;ay];
% beta    = atan(vy/vx); 
% Yaw     = [x(5,end);x(6,end)];
% X       = x(7,end);
% Y       = x(9,end);
% XY      = [X;Y];
% VXY     = [x(8,end);x(10,end)];
% PSI     = [x(11,end);x(12,end)];



%###################################################################
%######################  Chassis Model   ###########################
%###################################################################

% tspan       = [0 ts];
% [t,x]       = ode45(@(t,x) odefcn(t,x,m,Iveh,a,b,c,Fx,Fy),tspan,x0);   %solve ODE, use default error size
% x           = x.';
% vx          = x(2,end);
% vy          = x(4,end);
% vcog        = [vx;vy];
% beta        = atan(vy/vx); 
% 
% % Initialize vectors for speed
% ax          = zeros(1,length(x));
% ay          = zeros(1,length(x));
% apsi        = zeros(1,length(x));
% Vx          = zeros(1,length(x));
% Vy          = zeros(1,length(x));
% for i = 1:length(x)
%     ax(i)   = (1/m)*(m*x(4,i)*x(6,i)+Fx(1)+Fx(2)+Fx(3)+Fx(4));
%     ay(i)   = (1/m)*(-m*x(2,i)*x(6,i)+Fy(1)+Fy(2)+Fy(3)+Fy(4));
%     apsi(i) = (1/Iveh)*(a*(Fy(1)+Fy(2))-b*(Fy(3)+Fy(4))+c*(-Fx(1)+Fx(2)-Fx(3)+Fx(4)));
%     Vx(i)   = x(2,i)*cos(x(9,i)) - x(4,i)*sin(x(9,i));
%     Vy(i)   = x(2,i)*sin(x(9,i)) + x(4,i)*cos(x(9,i));
% end
% 
% Yaw         = [x(5,end);x(6,end);apsi(end)];  %[position; velocity; accel.];
% acog        = [ax(end);ay(end)];
% XY          = [x(7,end);x(8,end)];
% VXY         = [Vx(end);Vy(end)];
% PSI         = [x(5,end);x(6,end)];    % [position; velocity];
% 
% % Define function describing nonlinear system
% function dx_dt = odefcn(t,x,m,Iveh,a,b,c,Fx,Fy)
% % states x = [x, vx, y, vy, psi, psidot, X, Y, PSI]
% dx_dt = [x(2);
%     (1/m)*(m*x(4)*x(6)+Fx(1)+Fx(2)+Fx(3)+Fx(4));
%     x(4);
%     (1/m)*(-m*x(2)*x(6)+Fy(1)+Fy(2)+Fy(3)+Fy(4));
%     x(6);
%     (1/Iveh)*(a*(Fy(1)+Fy(2))-b*(Fy(3)+Fy(4))+c*(-Fx(1)+Fx(2)-Fx(3)+Fx(4)));
%     x(2)*cos(x(5))-x(4)*sin(x(5));
%     x(2)*sin(x(5))+x(4)*cos(x(5));
%     x(6)];
% end    

%       x0 = [vx0; vy0; psi0; psidot0; X0; Y0]
vx0             = x0(1,1);
vy0             = x0(2,1);
psi0            = x0(3,1);
psidot0         = x0(4,1);
X0              = x0(5,1);
Y0              = x0(6,1);

A = [(1/m)*(m*vy0*psidot0 + Fx(1) + Fx(2) + Fx(3) + Fx(4));
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