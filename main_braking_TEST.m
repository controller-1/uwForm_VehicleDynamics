%% main_braking
% Purpose:
%   main file to simulate vehicle dynamics behavior under braking scenarios, throttle,
%   and minor steering maneuvers
% Inputs:
%   Steering angle, brake torque, throttle torque
%
% Limitations of the model:
%   Many of the limitations take form of simplifying assumptions within
%   each function block of the vehicle model. See reference text for
%   additional detail:
%   **ADD REPORT CITATION HERE ONCE CREATED
%
% K. Barreto
% Version: 9/2/2021


%###################################################################
%########  Simulation Parameters & Initial Conditions  #############
%###################################################################
load('T32CarParamObj.mat');

tfinal          = 5;                    % simulation time [sec.]
delt            = 0.0001;               % simulation time step [sec.]
t               = 0:delt:tfinal;        % time sequence

% Assume vehicle starts at steady state velocity > 0 travelling in straight
% line

Vcogx0          = 30;                   % initial vehicle COG longit. speed [m/s]
Vcogy0          = 0;                    % initial vehicle COG lateral speed [m/s]
Acogx0          = 0;                    % initial vechile COG longit. accel. [m/s^2]
Acogy0          = 0;                    % initial vehicle COG lateral accel. [m/s^2]
Beta0           = 0;                    % initial body side slip angle [rad]
YawR0           = 0;                    % initial yaw rate of vehicle around COG [rad/s]
YawRAcc0        = 0;                    % initial yaw rate accel. of vehicle COG [rad/s^2]

X0              = 0;                    % initial X-axis Coord of vehicle COG w.r.t. Earth fixed reference frame [m]
Y0              = 0;                    % initial Y-axis Coord of vehicle COG w.r.t. Earth fixed reference frame [m]
PsiA0           = 0;                    % initial angle between x-axis of E.Fixed reference frame and vehicle frame [rad]

Bd              = [0.00001;0.00001];    % Wheel speed damping coefficient
Iveh            = 240;                  % vehicle moment of inertia


%###################################################################
%##################  Generate Input Profiles  ######################
%###################################################################
%%%% Brake Torque Profile
MeasuredFlag=inptdf('\n\nChoose if BRAKE TORQUE input: 1. Yes, 2. No,[1]',1);
disp(sprintf('\nMax braking torque = %-10.4e N*m',Brakes.maxBrakeT));
if MeasuredFlag == 1
    TStart=inptdf('\n\nEnter Brake start time in (sec.), (1)', 1);
    TStop=inptdf('\n\nEnter Brake stop time in (sec.), (2)', 2);
    Amp=inptdf('\n\nEnter Brake torque peak amplitude in (Nm), (-50)', -50); 
    Bias=inptdf('\n\nEnter brake bias, (0.72)',car.brakeBias);
    W_TB = BP6(t,TStart,TStop,Amp,Bias);
elseif MeasuredFlag == 2
    W_TB = BP6(t,1,2,0,car.brakeBias);  
end

%%%% Steering Profile
MeasuredFlag2=inptdf('\n\nChoose if WHEEL STEERING ANGLE input: 1. Yes, 2. No, [1]',1);
disp(sprintf('\nMax wheel angle = %-10.4e deg',car.maxWangle));
if MeasuredFlag2 == 1
    TStart=inptdf('\n\nEnter Steering start time in (sec.), (1)', 1);
    TStop=inptdf('\n\nEnter Steering stop time in (sec.), (2)', 2);
    Amp=inptdf('\n\nEnter Steering angle peak amplitude in (deg), (1)', 1);     

    FW_Ang = SP3(t,TStart,TStop,Amp); % Load custom brake torque profile
elseif MeasuredFlag2 == 2
    FW_Ang = SP2(t,1,2,0);
end

%%%% Throttle Profile
%% 


%###################################################################
%######################  Run Simulation  ###########################
%###################################################################
w0                  = (Vcogx0)/(car.wheel_dia/2); 
Vcogx0              = w0*(car.wheel_dia/2);                 % Re-compute initial vcogx0 to remove machine accuracy error 
w_w                 = zeros(4,length(t));                   % wheel angular velocity [rad/s]
w_w(:,1)            = w0;
w_acc               = zeros(4,length(t));                   % wheel angular acceleration [rad/s^2]
w_acc(:,1)          = 0;
Vcog                = zeros(2,length(t));                   % vehicle COG velocity [m/s]
Vcog(:,1)           = [Vcogx0;Vcogy0];
VcogMag             = zeros(1,length(t));                   % magnitude of vehicle COG velocity [m/s]
VcogMag(1,1)        = sqrt(Vcogx0^2 + Vcogy0^2);
Acog                = zeros(2,length(t));                   % vehicle COG acceleration [m/s^2]
Acog(:,1)           = [Acogx0;Acogy0];
Beta                = zeros(1,length(t));                   % vehicle body side slip angle
Beta(1,1)           = Beta0;
YawP                = zeros(1,length(t));                   % initial vehicle angular position [rad]
YawP(1,1)           = PsiA0;
YawR                = zeros(1,length(t));                   % vehicle yaw rate [rad/s]
YawR(1,1)           = YawR0;
YawRAcc             = zeros(1,length(t));                   % vehicle yaw acceleration [rad/s^2]
YawRAcc(1,1)        = YawRAcc0;
XYc                  = zeros(2,length(t));                   % vehicle COG coordinates w.r.t. earth fixed reference frame [m]
XYc(:,1)             = [X0;Y0];
VX0                 = Vcogx0*cos(PsiA0)-Vcogy0*sin(PsiA0);
VY0                 = Vcogx0*sin(PsiA0)+Vcogy0*cos(PsiA0);
V                   = zeros(2,length(t));                   % vehicle COG velocity w.r.t. fixed reference frame [m/s]
V(:,1)              = [VX0;VY0];
PsiA                = zeros(1,length(t));                   % angle of vehicle x-axis w.r.t. fixed ref. frame x-axis [rad]
PsiA(1,1)           = PsiA0;
Fl                  = zeros(4,length(t));                   % longitudinal tire forces [N]
Fc                  = zeros(4,length(t));                   % lateral tire forces [N]
wgpv                = zeros(4,length(t));                   % wheel ground point velocity [m/s]
wgpv_comp           = zeros(4,length(t));
rewv                = zeros(4,length(t));                   % rotational equivalent wheel velocity [m/s]
cam_ang             = [car.staticCamberF,car.staticCamberR];% static camber angles [deg]
a_w                 = zeros(4,length(t));                   % tire side slip angle [rad]
lSR                 = zeros(4,length(t));                   % longitudinal slip ratio
FX                  = zeros(4,length(t));                   % equivalent vehicle x-axis tire forces [N]
FY                  = zeros(4,length(t));                   % equivalent vehicle y-axis tire forces [N]
R                   = car.wheel_dia/2;                      % unloaded wheel/tire radius

Fzw                  = zeros(4,length(t));                  % dynamic vertical wheel forces
FzCOP               = zeros(4,length(t));                   % Aerodynamic Forces @ COP [N]
LongWT              = zeros(1,length(t));                   % Longitudinal Weight transfer
% Static vertical tire forces before load transfer and aero
% FL_static           = car.mass * g * (car.cgdistance * car.lateralCGDistance);
% FR_static           = car.mass * g * (car.cgdistance * (1 - car.lateralCGDistance));
% RL_static           = car.mass * g * (1 - car.cgdistance) * car.lateralCGDistance;
% RR_static           = car.mass * g * (1 - car.cgdistance) * (1 - car.lateralCGDistance);
FL_static           = car.mass * g * (car.cgdistance * 0.5);
FR_static           = car.mass * g * (car.cgdistance * (1 - 0.5));
RL_static           = car.mass * g * (1 - car.cgdistance) * 0.5;
RR_static           = car.mass * g * (1 - car.cgdistance) * (1 - 0.5);

%wtest               = zeros(1,length(t));
%Vtest               = zeros(4,length(t));
Fl2                  = zeros(4,length(t));                   % longitudinal tire forces using only long. tire stiffness [N]

cog2cop             = zeros(4,length(t));                   % Geometric distance from COG to each wheel COP
copAng              = zeros(4,length(t));                   % Angle between axis and geometric distance cog2cop


for i = 1:length(t)
    if Vcog(1,i)>0

        %wtest(1,i) = Vcog(1,i)/R; 
        %Vtest(:,i) = w_w(:,i).*R;

        %Compute Aerodynamic vertical forces
        aeroLift        = aeroForce(car.cL, car.frontArea, Vcog(1,i));
        aeroDrag        = aeroForce(car.cD, car.frontArea, Vcog(1,i));
        %aeroLift        = aeroForce(-10, car.frontArea, Vcog(1,i));
        %aeroDrag        = aeroForce(5, car.frontArea, Vcog(1,i));
        frontFZAero     = (-(aeroLift * car.COPdistance) - aeroDrag * car.COPHeight / car.wheelbase);
        rearFZAero      = (-(aeroLift * (1 - car.COPdistance)) + aeroDrag * car.COPHeight / car.wheelbase);
        FzCOP(:,i)      = [aeroLift;aeroDrag;frontFZAero;rearFZAero];
        %FzCOP(:,i)  = [frontFZAero,rearFZAero];
       
        %Compute dynamic wheel vertical forces
        %LongWT(i) = car.mass * car.cgheight * (-Acog(1,i) * g) /%car.wheelbase;
        %Long. Weight Transfer using G acceleration
        LongWT(i)          = car.mass * car.cgheight * (-Acog(1,i)) / car.wheelbase; %Long. Weight Transfer
        FLFZCurrent     = FL_static + (LongWT(i) / 2) + (frontFZAero / 2);
        FRFZCurrent     = FR_static + (LongWT(i) / 2) + (frontFZAero / 2);
        RLFZCurrent     = RL_static - (LongWT(i) / 2) + (rearFZAero / 2);
        RRFZCurrent     = RR_static - (LongWT(i) / 2) + (rearFZAero / 2);
        Fz              = [FLFZCurrent,FRFZCurrent,RLFZCurrent,RRFZCurrent];
        Fzw(:,i)        = Fz.';

        % Compute tire forces
        if i+1 <= length(t)
            [Fl(:,i),Fc(:,i),lSR(:,i),wgpv(:,i),rewv(:,i),a_w(:,i),cog2cop(:,i),copAng(:,i),wgpv_comp(:,i)]= Pacejka_Tire_Forces(car.tire,car.roadCoefficient,...
                0,cam_ang,Beta(1,i),VcogMag(1,i),YawR(1,i),Fz,FW_Ang(i),car.Ftrack,car.Rtrack,car.cg2faxle,car.cg2raxle,R,w_w(:,i).');
            
            % Simplified long. force calculation for comparison
            Cx = 23782.44; %long tire stiffness
            Fl2(:,i) = Cx*lSR(:,i);
        end

        % Compute wheel speeds
        if i+1 <= length(t)
            %w_w(:,i+1)  = wheel_speed(Fl(:,i),0,W_TB(:,i),Bd,Brakes.IWheel,(car.wheel_dia/2),delt,w_w(:,i));
            %w_w(:,i+1)  = wheel_speed(Fl(:,i),0,W_TB(:,i),Bd,0.15,(car.wheel_dia/2),delt,w_w(:,i));
            [w_w(:,i+1),w_acc(:,i+1)]  = wheel_speed_TEST(Fl(:,i),0,W_TB(:,i),Bd,0.15,R,delt,w_w(:,i));
        end        
        
        % Compute transformed axis component forces from long./lateral tire forces
        [fx,fy]= TireForces_COG(Fl(:,i),Fc(:,i),FW_Ang(i),car.maxWangle);
        for j = 1:4
            FX(j,i) = fx(j,1);
            FY(j,i) = fy(j,1);
        end
       
        % Simulate Nonlinear 2D Chassis model
        if i+1 <= length(t)

            x0 = [Vcog(1,i);Vcog(2,i);YawP(1,i);YawR(1,i);XYc(1,i);XYc(2,i)];
            [vcog,acog,beta,yaw,xym,vxym,psi] = PlantModel_2D_TEST(FX(:,i).',FY(:,i).',x0,car.mass,Iveh,car.cg2faxle,car.cg2raxle,car.wheelbase,delt);
            Vcog(:,i+1)     = [vcog(1,1);vcog(2,1)];
            Acog(:,i+1)     = [acog(1,1);acog(2,1)];
            Beta(1,i+1)     = beta;
            YawP(1,i+1)     = yaw(1,1);
            YawR(1,i+1)     = yaw(2,1);
            YawRAcc(1,i+1)  = yaw(3,1);
            XYc(:,i+1)      = [xym(1,1);xym(2,1)];
            V(:,i+1)        = [vxym(1,1);vxym(2,1)];
            PsiA(1,i+1)     = psi(1,1);
            
            VcogMag(1,i+1)  = sqrt(vcog(1,1)^2 + vcog(2,1)^2);
        end
    elseif Vcog(1,i)<=0  % Stop simulation when vehicle begins to move in reverse. Model does not like this. 
        break
    end
end


%%
%###################################################################
%##################  Results Visualization   #######################
%###################################################################

figure('Name','Output: Velocity(COG) x-comp.','NumberTitle','off')
plot(t,Vcog(1,:))
title('Velocity Vx')
%hold on
%plot(t,Vtest(1:4,:))
%legend('Vcog','vfl from wheel_speed','vfr from wheel_speed','vrl from wheel_speed','vrr from wheel_speed')
legend('Vcog')
xlabel('t')
ylabel('Velocity (m/s)')
ylim([min(Vcog(1,:))-0.2 max(Vcog(1,:)+0.2)])
hold on
y1 = ones(1,length(t)) * max(Vcog(1,:));
plot(t,y1,'--k')

figure('Name','Output: Velocity(COG) y-comp.','NumberTitle','off')
plot(t,Vcog(2,:))
title('Velocity Vy')
%hold on
%plot(t,Vtest(1:4,:))
%legend('Vcog','vfl from wheel_speed','vfr from wheel_speed','vrl from wheel_speed','vrr from wheel_speed')
legend('Vcog')
xlabel('t')
ylabel('Velocity (m/s)')
ylim([min(Vcog(2,:))-0.2 max(Vcog(2,:)+0.2)])
hold on
y1 = ones(1,length(t)) * max(Vcog(2,:));
plot(t,y1,'--k')

figure('Name','Output: Accel(COG) x-comp.','NumberTitle','off')
plot(t,Acog(1,:))
title('Accel Ax')
xlabel('t (sec.)')
ylabel('Accel (m/s^2)')
ylim([min(Acog(1,:))-0.05, max(Acog(1,:))+0.05])
grid on

figure('Name','Output: Accel(COG) y-comp.','NumberTitle','off')
plot(t,Acog(2,:))
title('Accel Ay')
xlabel('t (sec.)')
ylabel('Accel (m/s^2)')
ylim([min(Acog(2,:))-0.05, max(Acog(2,:))+0.05])
grid on

figure('Name','Output: Longitudinal Slip Ratio','NumberTitle','off')
plot(t,lSR(1:4,:))
title('Long. Slip Ratio')
legend('fl','fr','rl','rr')
xlabel('t (sec.)')
ylabel('Slip Ratio (dimensionless)')
grid on

figure('Name','Output: Long. Tire Force','NumberTitle','off')
subplot(2,1,1)
plot(t,Fl(1:4,:))
title('Long. Tire Force')
legend('fl','fr','rl','rr')
xlabel('t (sec.)')
ylabel('Force (N)')
grid on

subplot(2,1,2)
plot(t,FX(1:4,:))
title('Tire Force X-coord (EFRF)')
legend('fl','fr','rl','rr')
xlabel('t (sec.)')
ylabel('Force (N)')
grid on

figure('Name','Output: Lateral Tire Force','NumberTitle','off')
subplot(2,1,1)
plot(t,Fc(1:4,:))
title('Lateral Tire Force')
legend('fl','fr','rl','rr')
xlabel('t (sec)')
ylabel('Force (N)')
grid on

subplot(2,1,2)
plot(t,FY(1:4,:))
title('Tire Force Y-coord (EFRF)')
legend('fl','fr','rl','rr')
xlabel('t (sec)')
ylabel('Force (N)')
grid on

figure('Name','Output: Wheel Ang. Velocities','NumberTitle','off')
plot(t,w_w(1:4,:))
title('Wheel Angular Velocities')
%hold on
%plot(t,wtest(1,:))
%legend('fl','fr','rl','rr','wtest(computed w/ PlantModel Vcog)')
legend('fl','fr','rl','rr')
xlabel('t (sec)')
ylabel('Angular Velocity (rad/s)')
grid on

figure('Name','Output: Wheel Ang. Accel','NumberTitle','off')
plot(t,w_acc(1:4,:))
title('Wheel Angular Accelerations')
legend('fl','fr','rl','rr')
xlabel('t (sec)')
ylabel('Angular Accel. (rad/s^2)')
grid on

figure('Name','Output: Aerodynamic Forces','NumberTitle','off')
plot(t,FzCOP(1:4,:))
title('Aerodynamic Forces')
legend('Total Lift @ COP','Total Drag','Front Axle Vertical Load','Rear Axle Vertical Load')
xlabel('t (sec.)')
ylabel('Force (N)')

% figure('Name','Input: Brake Torque Profile','NumberTitle','off')
% plot(t,W_TB(1:4,:))
% title('Brake Torque')
% legend('BT_fl','BT_fr','BT_rl','BT_rr')
% xlabel('t (sec.)')
% ylabel('Torque (Nm)')
% grid on 
% ylim([min(W_TB(1,:))-0.05*min(W_TB(1,:)) max(W_TB(1,:))+0.05*max(W_TB(1,:))])

figure('Name','Input: Wheel Steering Angle Profile','NumberTitle','off')
plot(t,FW_Ang)
title('Wheel Steering Angle Profile')
xlabel('t (sec.)')
ylabel('Angle (deg)')
grid on


% Find peak magnitude of Fl*R and corresponding index
[FlRMax, flrind] = max(Fl(1,:).*-R);

figure('Name','Output: FL Wheel Dynamics','NumberTitle','off')
%plot(t,Fl(1:4,:).*R,t,W_TB(1:4,:).*-1,t,w_acc(1:4,:))
%legend('Fl*R_fl','Fl*R_fr','Fl*R_rl','Fl*R_rr','BT_fl','BT_fr','BT_rl','BT_rr','a_fl','a_fr','a_rl','a_rr')
yyaxis left
plot(t,Fl(1,:).*(-R),'-k',t,W_TB(1,:),'-r',t,w_acc(1,:).*1,'-b')
hold on
plot(ones(1,length(t)).*t(flrind),linspace(-50,250,length(t)),'--k')
xlabel('t (sec.)')
ylabel('Tire Force Torque(N*m), Brake Torque(N*m), Wheel Acc. (rad/s^2)')

yyaxis right 
ylabel('LSR')
hold on
plot(t,lSR(1,:).*-1,'-g')
grid on
legend('Fl*R_{fl}','Tbrake_{fl}','a_{fl}','Vertical Line','LSR (flipped)')
title('Front Left Wheel Dynamics Parameters')

% Find peak magnitude of LSR and corresponding index
[lsrMax,ind ] = min(lSR(1,:));

% Compute slopes of above plots
ddtFlR = zeros(4,length(t)-1);
ddtW_TB = zeros(4,length(t)-1);
ddtLSR = zeros(4,length(t)-1);
ddtWGPV = zeros(4,length(t)-1);
ddtREWV = zeros(4,length(t)-1);
for i = 1:(length(t)-1)
    ddtFlR(:,i) = (Fl(:,i+1).*-R - Fl(:,i).*-R)/delt;
    ddtW_TB(:,i) = (W_TB(:,i+1)-W_TB(:,i))/delt;
    ddtLSR(:,i) = (lSR(:,i+1).*-1 - lSR(:,i).*-1)/delt;
    ddtWGPV(:,i) = (wgpv(:,i+1)-wgpv(:,i))/delt;
    ddtREWV(:,i) = (rewv(:,i+1)-rewv(:,i))/delt;
end
ddtDiff = ddtW_TB(1,1:1000:end)-ddtFlR(1,1:1000:end);
tDiff = t(1:1000:end-1);

figure('Name','Output: FL Wheel Dynamics Slopes','NumberTitle','off')
yyaxis left
plot(t(1:end-1),ddtFlR(1,:),t(1:end-1),ddtW_TB(1,:),tDiff,ddtDiff,'--xr')
ylabel('dFlR/dt (Nm/s), dTbrake/dt (Nm/s)')
xlabel('t (sec.)')
xlim([0.9 2.1])
yyaxis right
plot(t(1:end-1),ddtLSR(1,:))
ylabel('dLSR/dt')
legend('Fl*R slope','T_brake slope','Fl*R and Tbrake slope Difference','LSR slope')
grid on

figure('Name','Output: FL Wheel Velocites for LSR Calculation','NumberTitle','off')
yyaxis left
plot(t,wgpv(1,:),'-b',t,rewv(1,:),'-r',t,Vcog(1,:),'--g')
hold on
plot(ones(1,length(t)).*1.5,linspace(13.4,15.5,length(t)),'--k')
ylabel('Velocities (m/s)')
%ylim([13.4 15.25])
xlabel('t (sec)')
grid on

yyaxis right
plot(t,lSR(1,:))
ylim([-4.5*10^-3 1*10^-3])
legend('WGPV_{fl}','REWV_{fl}','COG Vx_{fl}','CenterLine','LSR_{fl}')
title('Front Left Wheel LSR Parameters')


figure('Name','Output: Wheel Ground Point Velocities (GPV)')
subplot(3,1,1)
plot(t,wgpv(1:4,:))
xlabel('t (sec.)')
ylabel('Velocity (m/s)')
legend('fl','fr','rl','rr')
title('Individual Wheel GPV')
grid on

subplot(3,1,2)
plot(t,wgpv_comp(1:2,:))
xlabel('t (sec.)')
ylabel('Velocity (m/s)')
legend('x-comp.','y-comp')
title('FL WGPV Components')
grid on

subplot(3,1,3)
plot(t,wgpv_comp(3:4,:))
xlabel('t (sec.)')
ylabel('Velocity (m/s)')
legend('x-comp.','y-comp')
title('FR WGPV Components')
grid on

figure('Name','Output: Tire Side Slip Angle','NumberTitle','off')
plot(t,rad2deg(a_w(1:4,:)))
legend('fl','fr','rl','rr')
xlabel('t (sec.)')
ylabel('Side Slip Angle (deg)')
title('Tire Side Slip Angle (deg)')
grid on

figure('Name','Ouput: Geometry of Wheel COP to Undercarriage COG')
subplot(2,1,1)
plot(t,cog2cop(1:4,:))
legend('fl','fr','rl','rr')
xlabel('t (sec.)')
ylabel('Distance: COG to Wheel COP (m)')

subplot(2,1,2)
plot(t,rad2deg(copAng(1:4,:)))
xlabel('t (sec.)')
ylabel('Angle: COP and Ref. Axis (deg)')


figure('Name','Output: Body Side Slip Angle','NumberTitle','off')
%plot(t,rad2deg(Beta))
plot(t,Beta)
xlabel('t (sec.)')
ylabel('Angle (rad)')
title('Body Side Slip Angle')
grid on

figure('Name','Output: Yaw Quantities','NumberTitle','off')
subplot(3,1,1)
plot(t,rad2deg(YawP))
xlabel('t (sec)')
ylabel('Angle (deg)')
title('Yaw Position Angle ')
grid on

subplot(3,1,2)
plot(t,rad2deg(YawR))
xlabel('t (sec.)')
ylabel('Ang. Velocity (deg/s)')
title('Yaw Rate')

subplot(3,1,3)
plot(t,rad2deg(YawRAcc))
xlabel('t (sec)')
ylabel('Ang. Accel. (deg/s^2)')
title('Yaw Rate Accel.')

figure('Name','Output: Wheels Total Vertical Loads','NumberTitle','off')
plot(t,Fzw(1:4,:))
legend('fl','fr','rl','rr')
xlabel('t (sec.)')
ylabel('Force (N)')
ylim([min(Fzw,[],'all')-0.05*min(Fzw,[],'all'),max(Fzw,[],'all')+0.05*max(Fzw,[],'all')]) 
title('Wheel Total Vertical Load')

figure('Name','Output: Long. Weight Transfer','NumberTitle','off')
plot(t,LongWT)
xlabel('t (sec)')
ylabel('Force (N)')
title('Longitudinal Weight Transfer')

figure('Name','Output: Vehicle COG Position','NumberTitle','off')
plot(XYc(1,:),XYc(2,:))
xlabel('X-position (m)')
ylabel('Y-position (m)')
title('Vehicle COG position on Earth')
