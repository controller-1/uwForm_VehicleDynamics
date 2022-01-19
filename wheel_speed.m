function [w,Wacc]= wheel_speed(Fl,Tc,Tb,Bd,Iw,R,ts,w0)
% Wheel model. Computes the wheel angular speed from the wheel dynamics 
% ODEs. 
% 
% Inputs:
% Fl; longit. tire forces from tire model computation, 4x1 array
%       Negative input --> decelerating
%       Positive input --> accelerating
% Tc; traction torque (throttle) from powertrain model, 4x1 array
%       input 0 (scalar) to completely turn off traction torque 
% Tb; brake torque from brake model, 4x1 array
%       Positive values expected
%       input 0 (scalar) to completely turn off brake torque
% Bd; damping coefficient acting on wheel angular speed
%       [front damping; rear damping];
%       Correlating factor (i.e. model drivetrain torque effects)
% Iw; wheel+tire moment of inertia
% R; radius of wheel+tire [m]
%       ** Future implement using individual wheel effective radius
% ts; simulation time [sec]
% w0; initial condition wheel angular speed [rad/sec], 4x1 column array
% 
% Outputs:
% w; vector containing individual wheel angular speeds
%     [wfl,wfr,wrl,wrr].T
%     fl/r - front left/right
%     rl/r - rear left/right
%     
% NOTES: The current algorithm can be used for AWD platform (Tc input on all 
% 4 wheels), FWD platform (Tc input front wheels only), or RWD (Tc input on 
% rear wheels). 
%
% K. Barreto
% MSME - University of Washington
% UW Formula
% 1/18/2022

if Tc == 0
    Tc = [0;0;0;0];
end
if Tb == 0
    Tb = [0;0;0;0];
end

Wacc = [(1/Iw)*(-Fl(1,1)*R + Tc(1,1) - Tb(1,1)) - Bd(1,1)*w0(1,1);
       (1/Iw)*(-Fl(2,1)*R + Tc(2,1) - Tb(2,1)) - Bd(1,1)*w0(2,1);
       (1/Iw)*(-Fl(3,1)*R + Tc(3,1) - Tb(3,1)) - Bd(2,1)*w0(3,1);
       (1/Iw)*(-Fl(4,1)*R + Tc(4,1) - Tb(4,1)) - Bd(2,1)*w0(4,1)];
w1 = w0(1,1) + Wacc(1,1)*ts;
w2 = w0(2,1) + Wacc(2,1)*ts;
w3 = w0(3,1) + Wacc(3,1)*ts;
w4 = w0(4,1) + Wacc(4,1)*ts;
w = [w1;w2;w3;w4];

end