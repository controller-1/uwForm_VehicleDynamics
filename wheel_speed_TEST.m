function [w,Wacc]= wheel_speed_TEST(Fl,Tc,Tb,Bd,Iw,R,ts,w0)
% Wheel model. Computes the wheel angular speed from the wheel dynamics 
% ODEs. 
% 
% Inputs:
% Fl; longit. tire forces from tire model computation, 4x1 array
%       Negative input --> decelerating
%       Positive input --> accelerating
% Tc; traction torque from powertrain model, 4x1 array
%       input 0 (scalar) to neglect traction torque 
% Tb; brake torque from brake model, 4x1 array
%       Positive values expected
% Bd; damping coefficient acting on wheel angular speed
%       [front damping; rear damping];
% Iw; wheel+tire moment of inertia
% R; radius of wheel+tire [m]
%       ** Future implement using individual wheel effective radius
% tfin; simulation time [sec]
% w0; initial condition wheel angular speed [rad/sec], 4x1 column array
% 
% Outputs:
% w; vector containing individual wheel angular speeds
%     [wfl,wfr,wrl,wrr].T
%     fl/r - front left/right
%     rl/r - rear left/right
%     
% NOTES: The current algorithm can be used for a 4WD vehicle given all four
% values in the input Tc (traction torque). However, the current setup
% assumes a RWD vehicle and utilizes a damping coefficient on the wheel
% speeds of the front two wheels (verify this!!). 
% Look into regenerative braking and how it might affect the wheel
% dynamics!
%
% K. Barreto
% 9/17/21

if Tc == 0
    Tc = [0;0;0;0];
end

%%%%Initial conditions
% only interested in wheel speed so set initial position to zero
% x0 = [ 0; w0(1,1); 0; w0(2,1); 0; w0(3,1); 0; w0(4,1)];
% Fl = Fl.*-1;
% ui = [(1/Iw)*(-Fl(1,1)*R + Tc(1,1) - Tb(1,1));
%     (1/Iw)*(-Fl(2,1)*R + Tc(2,1) - Tb(2,1));
%     (1/Iw)*(-Fl(3,1)*R + Tc(3,1) - Tb(3,1));
%     (1/Iw)*(-Fl(4,1)*R + Tc(4,1) - Tb(4,1))];
% 
% c = -Bd(1,1)/Iw;
% c2 = -Bd(2,1)/Iw;
% A = [ 0, 1, 0, 0, 0, 0, 0, 0;
%       0, c, 0, 0, 0, 0, 0, 0;
%       0, 0, 0, 1, 0, 0, 0, 0;
%       0, 0, 0, c, 0, 0, 0, 0;
%       0, 0, 0, 0, 0, 1, 0, 0;
%       0, 0, 0, 0, 0, c2, 0, 0;
%       0, 0, 0, 0, 0, 0, 0, 1;
%       0, 0, 0, 0, 0, 0, 0, c2];
% B = [ 0, 0, 0, 0;
%       1, 0, 0, 0;
%       0, 0, 0, 0;
%       0, 1, 0, 0;
%       0, 0, 0, 0;
%       0, 0, 1, 0;
%       0, 0, 0, 0;
%       0, 0, 0, 1];
% C = zeros(4,8); %Initialize output matrix
% C(1,2) = 1; C(2,4) = 1; C(3,6) = 1; C(4,8) = 1; %Select outputs
% t = linspace(0,tfin,10);
% u = repmat(ui.', numel(t), 1);
% sysSS = ss(A,B,C,0);
% yout = lsim(sysSS,u,t,x0);
% w = [yout(end,1);yout(end,2);yout(end,3);yout(end,4)];


%Fl = Fl.*-1;
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