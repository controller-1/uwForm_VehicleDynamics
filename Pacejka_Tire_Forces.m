function [Fl,Fc,lSR,WGPV,REWV,a_w,r,nu,uw_comp]=Pacejka_Tire_Forces(tire,mu,cond2,cam_ang,beta_rad,vcog,yawr,Fz,dw,bf,br,lf,lr,R,w)
% Returns the longitudinal and lateral tire forces of all 4 wheels 
% using the Pacejka tire model(Magic Formula).
% 
% Inputs:
% tire; tire model
% mu; externally supplied road coefficient
% cond2; include caster yes/no?
%         0 - no
%         1 - yes
%       nL; dynamic caster
%       nS; lateral shift (neglected)
% cam_ang; static camber angle [deg], 1x2 array [front,rear]
%       negative = lean inwards
%       positive = lean outwards
%       *camber dynamics neglected
%       NOTE: aka inclination angle (IA)
% beta_rad; body side slip angle [rad]
% vcog; magnitude of velocity of vehicle COG [m/s]
% yawr; yaw rate of vehicle around COG [rad/s]
% Fz; static vehicle vertical force per wheel, 1x4 array [N]
% dw; wheel turn angle (dont confuse with steering wheel angle) [deg]
% bf; front axle wheel distance [m]
% br; rear axle wheel distance [m]
% lf; COG to front axle distance [m]
% lr; COG to rear axle distance [m]
% R; static wheel/tire radius [m]
%       ** Future work: implement effective tire/wheel radius
% w; wheel angular speed [rad/s], 1x4 array
% 
% Outputs:
% Fl; longitudinal tire force, 1x4 array [N]
% Fc; lateral tire force, 1x4 array [N]
% lSR; longitudinal slip ratio [dimensionless]
% WGPV; wheel ground point velocity [m/s]
% REWV; rotational equivalent wheel velocity [m/s]
% 

% References: 
% [1] "Tire and Vehicle Dynamics" by Hans Pacejka 3rd Edition
% [2] "Automotive Control Systems" by Uwe Kiencke and Lars Nielsen
% [3] "Combined Longitudinal and Lateral Control for Automated Vehicle Guidance" by
%     Rachid Attia, Rodolfo Orjuela, and Michel Basset


% K.Barreto
% MSME - University of Washington
% UW Formula
% 1/18/2022


% disp(' ')
% disp('*****************************************************')
% disp('***       Tire Forces via Pacejka Model           ***')
% disp('*****************************************************')

dw_rad = deg2rad(dw);

%###################################################################
%##############  Caster Parameters Computation   ###################
%###################################################################
l_0 = -0.03;    % caster parameter [m]
l_1 =  0.12;    % caster parameter [m]
Fz_0= 5000;     % Nominal vertical force at wheel contact [N]
% See Table 8.2 for the above universal parameters
if cond2 == 0
    nL_fl = 0;
    nL_fr = 0;
    nL_rl = 0;
    nL_rr = 0;
elseif cond2 == 1
    nL_fl = (1/2)*(l_0 + l_1*(Fz(1,1)/Fz_0));
    nL_fr = (1/2)*(l_0 + l_1*(Fz(1,2)/Fz_0));
    nL_rl = (1/2)*(l_0 + l_1*(Fz(1,3)/Fz_0));
    nL_rr = (1/2)*(l_0 + l_1*(Fz(1,4)/Fz_0));
end

%###################################################################
%##############  Wheel-Ground Point Velocities   ###################
%###################################################################

% Compute distances from COG to wheel ground contact points [m]
r_fl = sqrt((lf-nL_fl*cos(dw_rad))^2 + ((bf/2)-nL_fl*sin(dw_rad))^2);
r_fr = sqrt((lf-nL_fr*cos(dw_rad))^2 + ((bf/2)+nL_fr*sin(dw_rad))^2);
r_rl = sqrt( (lr+nL_rl)^2 + (br/2)^2);
r_rr = sqrt( (lr+nL_rr)^2 + (br/2)^2);
r    = [r_fl,r_fr,r_rl,r_rr];
% Angles from COG frame to wheel ground contact points [rad]
nu_fl = atan(((bf/2)-nL_fl*sin(dw_rad))/(lf-nL_fl*cos(dw_rad)));
nu_fr = atan((lf-nL_fr*cos(dw_rad))/((bf/2)+nL_fr*sin(dw_rad)));
nu_rl = atan((lr+nL_rl)/(br/2));
nu_rr = atan((br/2)/(lr+nL_rr));
nu    = [nu_fl,nu_fr,nu_rl,nu_rr];
% wheel ground contact point velocities, magnitudes [m/s]
uw_fl = sqrt((vcog*cos(beta_rad)-yawr*r_fl*sin(nu_fl))^2 + ...
    (vcog*sin(beta_rad) + yawr*r_fl*cos(nu_fl))^2);
uw_fr = sqrt((vcog*cos(beta_rad)+yawr*r_fr*cos(nu_fr))^2 + ...
    (vcog*sin(beta_rad) + yawr*r_fr*sin(nu_fr))^2);
uw_rl = sqrt((vcog*cos(beta_rad)-yawr*r_rl*cos(nu_rl))^2 + ...
    (vcog*sin(beta_rad) - yawr*r_rl*sin(nu_rl))^2);
uw_rr = sqrt((vcog*cos(beta_rad)+yawr*r_rr*sin(nu_rr))^2 + ...
    (vcog*sin(beta_rad) - yawr*r_rr*cos(nu_rr))^2);
uw = [uw_fl,uw_fr,uw_rl,uw_rr];

uw_fl_x = vcog*cos(beta_rad)-yawr*r_fl*sin(nu_fl);
uw_fl_y = vcog*sin(beta_rad) + yawr*r_fl*cos(nu_fl);
uw_fr_x = vcog*cos(beta_rad)+yawr*r_fr*cos(nu_fr);
uw_fr_y = vcog*sin(beta_rad) + yawr*r_fr*sin(nu_fr);
uw_comp = [uw_fl_x;uw_fl_y;uw_fr_x;uw_fr_y];

%###################################################################
%##############  Tire Side Slip Angle Calcul.   ###################
%###################################################################
a_fl = -dw_rad - atan((vcog*beta_rad+yawr*r_fl*cos(nu_fl))/...
    (vcog-yawr*r_fl*sin(nu_fl)));   %side slip angle [rad]
a_fr = -dw_rad - atan((vcog*beta_rad+yawr*r_fr*sin(nu_fr))/...     
    (vcog+yawr*r_fr*cos(nu_fr)));
a_rl = atan((vcog*beta_rad-yawr*r_rl*sin(nu_rl))/...
    (vcog-yawr*r_rl*cos(nu_rl)));
a_rr = atan((vcog*beta_rad-yawr*r_rr*cos(nu_rr))/...
    (vcog+yawr*r_rr*sin(nu_rr)));
a_w_nocam = [a_fl,a_fr,a_rl,a_rr];

cam_ang1 = [cam_ang(1,1),-cam_ang(1,1),cam_ang(1,2),-cam_ang(1,2)];

a_w = zeros(1,4);
for i = 1:length(a_w_nocam)
    a_w(1,i)=a_w_nocam(1,i);
end


%###################################################################
%###########  Tire Longit.Slip Ratio Computation   #################
%###################################################################

eps = 0.0001;          %small quantity to avoid singularity (such as standstill scenarios)
vR_fl = R*w(1,1);   %rotational equivalent wheel velocity
vR_fr = R*w(1,2);
vR_rl = R*w(1,3);
vR_rr = R*w(1,4);
vR = [vR_fl,vR_fr,vR_rl,vR_rr];


LSR = zeros(1,4);
for i = 1:4
    LSR(1,i) = -(uw(1,i)*cos(a_w(1,i)) - vR(1,i))/abs(uw(1,i)*cos(a_w(1,i))+eps*sign(uw(1,i)));
end

%###################################################################
%#######################  Tire Forces   ############################
%###################################################################

Fl = zeros(1,4);
Fc = zeros(1,4);
for i = 1:length(Fl)
    Fl(1,i) = Pacejka(tire,'fx',-Fz(1,i),rad2deg(a_w(1,i)),LSR(1,i),cam_ang1(1,i),mu,vR(1,i),0);
    Fc(1,i) = Pacejka(tire,'fy',-Fz(1,i),rad2deg(a_w(1,i)),LSR(1,i),cam_ang1(1,i),mu,vR(1,i),0);
end
Fl = Fl.';
Fc = Fc.';
lSR = LSR.';
WGPV = uw.';
REWV = vR.';
a_w = a_w.';
r = r.';
nu = nu.';
end