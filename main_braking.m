%% main_braking
% Purpose:
%   main file to simulate vehicle dynamics behavior under braking scenarios
%   and minor steering maneuvers
% Inputs:
%   Steering angle, brake torque 
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

tfinal          = 5;                   % simulation time [sec.]
delt            = 0.001;                 % simulation time step [sec.]
t               = 0:delt:tfinal;        % time sequence

% Assume vehicle starts at steady state velocity > 0 travelling in straight
% line
Vcogx0          = 15;                   % initial vehicle COG longit. speed [m/s]
Vcogy0          = 0;                    % initial vehicle COG lateral speed [m/s]
Acogx0          = 0;                    % initial vechile COG longit. accel. [m/s^2]
Acogy0          = 0;                    % initial vehicle COG lateral accel. [m/s^2]
Beta0           = 0;                    % initial body side slip angle [rad]
YawR0           = 0;                    % initial yaw rate of vehicle around COG [rad/s]
YawRAcc0        = 0;                    % initial yaw rate accel. of vehicle COG [rad/s^2]

X0              = 0;                    % initial X-axis Coord of vehicle COG w.r.t. Earth fixed reference frame [m]
Y0              = 0;                    % initial Y-axis Coord of vehicle COG w.r.t. Earth fixed reference frame [m]
PsiA0           = pi/4;                 % initial angle between x-axis of E.Fixed reference frame and vehicle frame [rad]

Bd              = [0.00;0.000];                    % Wheel speed damping coefficient 0.0031


Iveh            = 15;                    % vehicle moment of inertia
% ^ replace quantity when resolve issue with car.MoIz

%###################################################################
%##################  Generate Input Profiles  ######################
%###################################################################
%%%% Brake Torque Profile
MeasuredFlag=inptdf('\n\nChoose desired BRAKE TORQUE input: 1. Sinusoid, 2. Square wave, 3. Load profile from source [1]',1);
disp(sprintf('\nMax braking torque = %-10.4e N*m',Brakes.maxBrakeT));
if MeasuredFlag == 1
    SinAmp=inptdf('\n\nEnter sinusoid amplitude in N*m, [-500;-500;-520;-520]',[-500;-500;-520;-520]);
    SinPeriod=inptdf('\n\nEnter sinusoid period in radians, [2*pi]',2*pi);
    W_TB = SinAmp.*sin(SinPeriod*t);
    for i = 1:length(W_TB)  %Filter out positive values from profile
        if W_TB(:,i)>0
            W_TB(:,i)=0;
        end
    end
elseif MeasuredFlag == 2
    SquareAmp=inptdf('\n\nEnter squarewave amplitude in N*m,[-693.3;-693.3;-725.0;-725.0]',Brakes.maxBrakeT);
    SquarePeriod=inptdf('\n\nEnter squarewave period in radians, [2*pi]',2*pi);
    W_TB = SquareAmp*square(SquarePeriod*t,0);
elseif MeasuredFlag == 3
    W_TB = BP1(t,0,car.brakeBias); % Load custom brake torque profile
end

%%%% Steering Profile
MeasuredFlag2=inptdf('\n\nChoose desired WHEEL STEERING ANGLE input: 1. Sinusoid, 2. Square wave, 3. Load profile from source [1]',1);
disp(sprintf('\nMax wheel angle = %-10.4e deg',car.maxWangle));
if MeasuredFlag2 == 1
    SinAmp=inptdf('\n\nEnter sinusoid amplitude in degrees, [5]',5);
    if SinAmp>car.maxWangle
        msg = 'Error: Input wheel angle greater than max angle';
        error(msg);
    end
    SinPeriod=inptdf('\n\nEnter sinusoid period in radians, [2*pi]',2*pi);
    FW_Ang = SinAmp.*sin(SinPeriod*t);
elseif MeasuredFlag == 2
    SquareAmp=inptdf('\n\nEnter squarewave amplitude in deg,[5]',5);
    SquarePeriod=inptdf('\n\nEnter squarewave period in radians, [2*pi]',2*pi);
    FW_Ang = SquareAmp*square(SquarePeriod*t,50);
elseif MeasuredFlag == 3
    FW_Ang = load(''); % Load custom brake torque profile
end


%###################################################################
%######################  Run Simulation  ###########################
%###################################################################
w0                  = Vcogx0/(car.wheel_dia/2); 
w_w                 = zeros(4,length(t));                   % wheel angular velocity [rad/s]
w_w(:,1)            = w0;
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
cam_ang             = [car.staticCamberF,car.staticCamberR];% static camber angles [deg]
lSR                 = zeros(4,length(t));                   % longitudinal slip ratio
FX                  = zeros(4,length(t));                   % equivalent vehicle x-axis tire forces [N]
FY                  = zeros(4,length(t));                   % equivalent vehicle y-axis tire forces [N]
R                   = car.wheel_dia/2;                      % unloaded wheel/tire radius
Fzw                  = zeros(4,length(t));                   % dynamic vertical wheel forces
FzCOP               = zeros(2,length(t));                   % Aerodynamic Forces @ COP [N]
% Static vertical tire forces before load transfer and aero
% FL_static           = car.mass * g * (car.cgdistance * car.lateralCGDistance);
% FR_static           = car.mass * g * (car.cgdistance * (1 - car.lateralCGDistance));
% RL_static           = car.mass * g * (1 - car.cgdistance) * car.lateralCGDistance;
% RR_static           = car.mass * g * (1 - car.cgdistance) * (1 - car.lateralCGDistance);
FL_static           = car.mass * g * (car.cgdistance * 0.5);
FR_static           = car.mass * g * (car.cgdistance * (1 - 0.5));
RL_static           = car.mass * g * (1 - car.cgdistance) * 0.5;
RR_static           = car.mass * g * (1 - car.cgdistance) * (1 - 0.5);

wtest               = zeros(1,length(t));
Vtest               = zeros(4,length(t));

for i = 1:length(t)
    if Vcog(1,i)>0

        wtest(1,i) = Vcog(1,i)/R; 
        Vtest(:,i) = w_w(:,i).*R;

        %Compute Aerodynamic vertical forces
        aeroLift        = aeroForce(car.cL, car.frontArea, Vcog(1,i));
        aeroDrag        = aeroForce(car.cD, car.frontArea, Vcog(1,i));
        frontFZAero     = (-(aeroLift * car.COPdistance) - aeroDrag * car.COPHeight / car.wheelbase);
        rearFZAero      = (-(aeroLift * (1 - car.COPdistance)) + aeroDrag * car.COPHeight / car.wheelbase);
        FzCOP(:,i)      = [aeroLift;aeroDrag];
       
        %Compute dynamic wheel vertical forces
        LongWT          = car.mass * car.cgheight * (-Acog(1,i) * g) / car.wheelbase; %Long. Weight Transfer
        FLFZCurrent     = FL_static + (LongWT / 2) + (frontFZAero / 2);
        FRFZCurrent     = FR_static + (LongWT / 2) + (frontFZAero / 2);
        RLFZCurrent     = RL_static - (LongWT / 2) + (rearFZAero / 2);
        RRFZCurrent     = RR_static - (LongWT / 2) + (rearFZAero / 2);
        %Fz              = [FLFZCurrent,FRFZCurrent,RLFZCurrent,RRFZCurrent];
        Fz = [FL_static,FR_static,RL_static,RR_static];
        Fzw(:,i)        = Fz.';
    
        % Compute tire forces
        [Fl(:,i),Fc(:,i),lSR(:,i)]= Pacejka_Tire_Forces(car.tire,car.roadCoefficient,1,cam_ang,Beta(1,i),VcogMag(1,i),YawR(1,i),Fz,FW_Ang(i),car.Ftrack,car.Rtrack,car.cg2faxle,car.cg2raxle,R,w_w(:,i).');
    %     for k = 1:4
    %         Fl(k,i) = fl(k,1);
    %         Fc(k,i) = fc(k,1);
    %     end
        
        % Compute wheel speeds
        if i+1 <= length(t)
            %w_w(:,i+1)  = wheel_speed(Fl(:,i),0,W_TB(:,i),Bd,Brakes.IWheel,(car.wheel_dia/2),delt,w_w(:,i));
            w_w(:,i+1)  = wheel_speed(Fl(:,i),0,W_TB(:,i),Bd,15,(car.wheel_dia/2),delt,w_w(:,i));
        end
        
        % Compute equivalent axis component forces from long./lateral tire forces
        [fx,fy]= TireForces_COG(Fl(:,i),Fc(:,i),FW_Ang(i),car.maxWangle);
        for j = 1:4
            FX(j,i) = fx(j,1);
            FY(j,i) = fy(j,1);
        end
       
        
        
        % Simulate Nonlinear 2D Chassis model
        if i+1 <= length(t)
            x0 = [0;Vcog(1,i);0;Vcog(2,i);YawP(1,i);YawR(1,i);XYc(1,i);XYc(2,i);PsiA(1,i)];
            [vcog,acog,beta,yaw,xym,vxym,psi] = PlantModel_2D(FX(:,i).',FY(:,i).',x0,car.mass,Iveh,car.cg2faxle,car.cg2raxle,car.wheelbase,delt);
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
    elseif Vcog(1,i)<=0
        break
    end
end


%%
%###################################################################
%##################  Results Visualization   #######################
%###################################################################

figure(1)
plot(t,Vcog(1,:))
title('Velocity Vx')
hold on
plot(t,Vtest(1:4,:))
legend('Vcog','vfl from wheel_speed','vfr from wheel_speed','vrl from wheel_speed','vrr from wheel_speed')
%legend('Vcog')
xlabel('t')
ylabel('Velocity (m/s)')

figure(2)
plot(t,Acog(1,:))
title('Accel Ax')

figure(3)
plot(t,lSR(1:4,:))
title('Long. Slip Ratio')
legend('fl','fr','rl','rr')
xlabel('t (sec.)')
ylabel('Slip Ratio (dimensionless)')

figure(4)
plot(t,Fl(1:4,:))
title('Long. Tire Force')
legend('fl','fr','rl','rr')
xlabel('t (sec.)')
ylabel('Force (N)')

figure(5)
plot(t,w_w(1:4,:))
title('Wheel Angular Velocities')
hold on
plot(t,wtest(1,:))
legend('fl','fr','rl','rr','wtest(computed w/ PlantModel Vcog)')
%legend('fl','fr','rl','rr')
xlabel('t (sec)')
ylabel('Angular Velocity (rad/s)')

figure(6)
plot(t,FzCOP(1:2,:))
title('Aerodynamic Forces @ COP')
legend('Lift','Drag')
xlabel('t (sec.)')
ylabel('Force (N)')