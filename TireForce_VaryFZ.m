%% Tire Forces from Pacejka() w/varying Fz

% Purpose of this script is to determine the change in longitudinal tire
% forces computed using Pacejka() for varying vertical wheel loading, Fz,
% while keeping all other variables constant.

load('T32CarParamObj.mat');

cam_ang = [0,0];
Beta = 0;
VcogMag = 14.4414;
yawr = 0;
dw = 0;
R = car.wheel_dia/2;
w_w = [62.6885,62.6885,63.0407,63.0407].';

Fz0 = [700,1200];
delFz1 = 1000;
n = 20;
FzFront = linspace(Fz0(1),Fz0(1)+delFz1,n);
FzRear = linspace(Fz0(2),Fz0(2)+delFz1,n);
Fz = [FzFront;FzFront;FzRear;FzRear];

Fl = zeros(4,length(n));
LSR = zeros(4,length(n));
for i = 1:n
    [Fl(:,i),Fc,LSR(:,i),WGPV,REWV,a_w]=Pacejka_Tire_Forces(car.tire,car.roadCoefficient,0,cam_ang,Beta,VcogMag...
       ,yawr,Fz(:,i).',dw,car.Ftrack,car.Rtrack,car.cg2faxle,car.cg2raxle,R,w_w.');
end


figure(1)
subplot(2,1,1)
yyaxis left
plot(1:1:n,Fl(1:4,:))
xlabel('n-th iteration')
ylabel('Long.Tire Force (N)')
title('Tire Force vs. Pure varying Vertical Loading')
grid on

yyaxis right
plot(1:1:n,Fz(1:4,:))
ylabel('Vertical Load (N)')
legend('Fl_{fl}','Fl_{fr}','Fl_{rl}','Fl_{rr}','Fz_{fl}','Fz_{fr}','Fz_{rl}','Fz_{rr}','Location','eastoutside')

subplot(2,1,2)
plot(1:1:n,LSR(1:4,:))
title('Long. Slip Ratio')
xlabel('n')
ylabel('Ratio (dimensionless)')
grid on
legend('LSR_{fl}','LSR_{fr}','LSR_{rl}','LSR_{rr}','Location','eastoutside')
