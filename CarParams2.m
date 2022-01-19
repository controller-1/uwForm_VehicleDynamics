%% FSAE CAR PARAMETERS
%
% Description:
%   This script to be used to define vehicle parameters and create data
%   structures to be used as inputs to functions. If any values are changed
%   this script needs to be re run to save an updated .mat file. This
%   script is based off of 'CarParams.m' written by Daniel Wageman.
%
% K. Barreto
% MSME - University of Washington
% UW Formula 
% 1/18/2022

clear all
clc

%% Car Settings

Name = ('T32_CarParams');       

tireCoP                         = 1;  % Approx grip of 10" tires compared to the 13"s.
g                               = 9.81;
namePrefix                      = ('');

% Tire data import
%(only TTC rnd 3 data is supported so far).
load ('Hoosier 20.5x7.0-13_7rim.mat');
% NEW Pacejka Format
% pacejkaFile                   = 'tires\R5_Hoosier 6.0-18.0-10 P12 LC0 Rim7.TIR';
pacejkaFile                     = 'LC0 Manual Coefficients with Mz.TIR';
tire                            = ImportTireData(pacejkaFile);


displacement                    = 450;                      % Engine displacement (CC)
otherCost                       = 0;                        % Any extra cost due to extra features
%motorType                       = 2;                        % Electric or combustion motor. 1 for combustion, 2 for electric
shiftTime                       = .1;                       % Est shift time (sec)
wheel_dia                       = .457;                     % Wheel diameter (m)
useCalculatedMass               = 1;                        % If 1: Calculate mass from individual components. If 0, use forced mass properties.
   

% Suspension Parameters
runPacejka                      = 1;                        % Run Pacjeka or MRA tire model
TLLTD                           = .50;                      % Total lateral load transfer distribution
wheelbase                       = 1.562;                    % Vehicle wheelbase (m)
Ftrack                          = 1.2446;                   % Front Track (m)
Rtrack                          = 1.1938;                   % Rear Track (m)
AVGtrackwidth                   = (Ftrack+Rtrack)/2;        % Avg of front and rear track width (m)
pitchCamberF                    = 1.32;                     % Front Pitch Camber Sensitivity
pitchCamberR                    = 1.5;                      % Rear Pitch Camber Sensitivity
rollCamberF                     = .662;                     % Front Roll Camber Sensitivity
rollCamberR                     = .532;                     % Rear Roll Camber Sensitivity
staticCamberF                   = -1.9;                     % Static Front Camber
staticCamberR                   = -1.5;                     % Static Rear Camber
wheelMoI                        = 591.83;                   % Wheel MoI in lb*in^2 (10") from SW
brakeBias                       = .72;
fixedBrakeBias                  = 1;

% Steering Parameters
stwheel_wheel_ratio             = 1.5;                      % steering wheel to wheel angle ratio
max_wangle                      = 35;                       % max wheel angle [deg]

% Aero Paramaters
COPdistance                     = .25;                       % Aero Center of Pressure (percent on front)
COPHeight                       = .508;                      % Aero Center of Pressure height (m)

% Electric Motor type parameters
numberOfMotors              = 2;
vehicleCOP                  = 1;                        % Parameter for "detuning" compared to max potential

numCells                    = 270;                      % Total # of cells
nominalCellV                = 3.7;                      % Volts
peakCellV                   = 4.2;                      % Volts
capacityCell                = 6.6;                       % Ah
nSeries                     = 90;                       % Number of cells in series
nParallel                   = 3;                        % Number of cells in parallel
contDischargeC              = 15;
peakDischargeC              = 20;
packV                       = nSeries*nominalCellV;     % Total Voltage
fullChargePackV             = nSeries*peakCellV;        % Total Voltage
packCapacity                = numCells*capacityCell*nominalCellV/1000;
contDischargeRate           = contDischargeC*capacityCell*nParallel;
peakDischargeRate           = peakDischargeC*capacityCell*nParallel;
contRechargeC               = 2;                        % 2C charge rate
peakRechargeC               = 4;                        % limit is 4C charge, no idea how long for and frequency allowed.

contRegenkW                 = contRechargeC * numCells * nominalCellV * capacityCell / 1000;   
                                    % max regen currently, dependent on pack size and cell selection.

peakNominalPower            = packV*peakDischargeRate/1000; % kW
peakMotorI                  = 200;                      % Amps
DRS_Enabled                 = 1;                        % Turn DRS evaluation on and off

cD                          = 1.44;                      % High coeff of BrkG.drag
cL                          = -3.05;                      % High coeff of lift
DRS_cD                      = .8993;                       % Actually CDA - Aero coeff of BrkG.drag when DRS is enabled
DRS_cL                      = -2.3835;                      % Actually CLA - Aero coeff of lift when DRS is enabled
rev_cD                      = 10/6.5;                     % Actually CDA - Aero coeff of BrkG.drag when going backward
rev_cL                      = 3.75/6.;                   % Actually CLA - Aero coeff of lift when going backward
frontArea                   = 1;% 1.49352;                 % Aero frontal area for BrkG.drag and lift calculation. Usually normalized to 1 by wind tunnel(m^2)
% Forced mass properties
if useCalculatedMass == 1
    % Individual Component Masses format: [mass,CGx,CGy,CGz]
    masses                  = xlsread('mass2.xlsx',1,'B2:E50');
    % Total mass of car
    mass = sum(masses(:, 1));

    % Moments of inertia and centrer of mass
    Ix      = masses(:, 1) .* masses(:, 2);
    Iy      = masses(:, 1) .* masses(:, 3);
    Iz      = masses(:, 1) .* masses(:, 4);
    CGx     = sum(Ix) / mass;
    CGy     = sum(Iy) / mass;
    CGz     = sum(Iz) / mass;

    % CG locations in relation to the rest of the car
    cgdistance          = (wheelbase - CGx) / wheelbase;
    avgTrack            = ((Ftrack + Rtrack) / 2);
    lateralCGDistance   = (avgTrack / 2 - CGy) / avgTrack;
    cgheight            = CGz;
    
    COG2Faxle               = cgdistance;               % COG to front axle length (m) **Verify that these are correct
    COG2Raxle               = wheelbase-cgdistance;     % COG to rear axle length (m)
else
    cgdistance              = .4056;                      % Mass percent on front
    lateralCGDistance       = .5;                       % Mass percent on left side of car
    cgheight                = 0.2728;                     % CG height (m)
    mass                    = 241.325;                  % Total vehicle mass + driver (kg)
    COG2Faxle               = 0.914;                    % COG to front axle length (m)
    COG2Raxle               = 1;                        % COG to rear axle length (m)
end
powerLimit                  = 80;%85;                   % ePower Limit (kW) 85 kw = 113.98, 80kw = 107.28
primary_reduction           = 4.345;                        % eMotor reduction
gears                       = [1];                      % Motor Gear
drivetrain_sprocket         = 1;                       % Teeth on rear sprocket
engine_sprocket             = 1;                       % Teeth on engine sprocket
redline                     = 6000;                     % Engine Redline
% Rough Emrax 188 Data
RPM                         = [7800     7200    6800    6400    6000    5000    4000    3000    2000    1000    1];
Torque                      = [77.143   80.571  82.286  83.143  84.857  86.143  88.286  88.714  89.571  90      90];
Torque                      = numberOfMotors * Torque; %Emrax True Peak
Power                       = (Torque * (2 * pi()) .* RPM / 60) / 1000;
% Replace any power values over peak power with peak power
for i=1:length(Power)
    if Power(1,i) > powerLimit
        Power(1,i) = powerLimit;
    end
end
% Add RPM values
% Re Calculate torque from adjusted power
Torque                      = 60 * Power * 1000 ./ (RPM * (2 * pi()));


% Brake Params
Brakes.Ambient          = 27 + 273;     % Ambient Temp [deg K]
Brakes.h                = 100;          % Convective Coef [W/m^2K]
Brakes.Area.Front       = .038;         % Front rotor area [m]
Brakes.Area.Rear        = .028;         % Rear rotor area [m]
Brakes.Mass.Front       = .46;          % Front rotor mass [kg]
Brakes.Mass.Rear        = .271;         % Rear rotor mass [kg]
Brakes.Cp.rotor         = 510;          % Rotor Specific Heat [J/kgK]
Brakes.Cp.pad           = 510;          % Rotor Specific Heat [J/kgK]
Brakes.IWheel           = .15;          % Wheel and Tire Moment of Inertia
Brakes.Radius           = .22;          % Wheel Radius
Brakes.Emissivity       = .28;          % From http://www.engineeringtoolbox.com/emissivity-coefficients-d_447.html, polished iron ranges .14 - .38
Brakes.sigma            = 5.6703E-8;    % Stefan-Boltzmann constant
Brakes.k.rotor          = 82;
Brakes.k.pad            = 8.7;
Brakes.k.area           = .007;
Brakes.Area.Caliper     = .015;
Brakes.Mass.Caliper     = 1.0;
Brakes.thickness.rotor  = .005;
Brakes.thickness.pad    = .010;

Brakes.Reffective       = .10;          %Effective braking radius [m]


 %% Global Setup
baseTrack               = 1.3;           % Base track width used to determine the course sizes

% Driver Coefficients
driverBrakingCoP        = .95;              % Scale maximum driver braking capacity - experimental
driverAcceleratingCoP   = .95;              % Scale maximum driver acceleration capacity - experimental
driverCorneringCoP      = .95;              % Scale maximum driver cornering capacity - experimental
roadCoefficient         = .69 * tireCoP;     % Road mu as suggested by MRA
Crr                     = 0.020;            % rolling resistance coefficient (est.)

GTol                    = .01;              % Iterative G force tolerance for solving brake, accel, and cornering G
maxItr                  = 50;               % Max GForce Iterations

wheel_circum            = wheel_dia * pi;   % Calculated wheel circumference feet

dt_ratio                = drivetrain_sprocket / engine_sprocket;   %Calculated drivetrain ratio
    
%% Top speed computation
% Top speed IN MPS, gearing limited

topSpeedAeroValid = 0;
while topSpeedAeroValid == 0
    topSpeedGear = ((redline / (primary_reduction * dt_ratio * gears(end))) / 60) * wheel_circum;

    topSpeedDragForce = aeroForce(cD,frontArea,topSpeedGear);
    topSpeedDragPower = topSpeedDragForce * topSpeedGear / 1000;
    fRollTires = Crr * mass * g;
    rollPowerLoss = ((Crr * (mass * g)* wheel_dia/2) * (2 * pi()) .* redline / 60) / 1000;

    maxTorque = eTEfficiency(interp1(RPM, Torque, redline, 'spline'),redline);

    maxIdealPower = (interp1(RPM, Torque, redline, 'spline') * (2 * pi()) .* redline / 60) / 1000;
    maxPower = (maxTorque * (2 * pi()) .* redline / 60) / 1000 - rollPowerLoss;

    if (abs(maxPower - topSpeedDragPower) > 0.5)
        if ((maxPower - topSpeedDragPower) >= 0)
            redline = redline + 25;
        elseif ((maxPower - topSpeedDragPower) < 0)
            redline = redline - 25;
        end
    else
        topSpeed = ((redline / (primary_reduction * dt_ratio * gears(end))) / 60) * wheel_circum;
        topSpeedAeroValid = 1;
    end 
end
    % when drag power + rolling resistance power loss + drivetrain/eTrain losses,
    % = power applied by wheels to max 80kW, we'll reach top speed
    
%%
Ncoarse = 20;
Nfine = 100;

% Unit conversions
SpeedRangeCoarseMPS     = linspace(0, topSpeed, Ncoarse);
SpeedRangeMPS           = linspace(0, topSpeed, Nfine);
SpeedRangeKPH           = SpeedRangeMPS * (3600 / 1000);          % Convert kph to m/s

SmoothRPM               = linspace(0, redline, 150);
SmoothTorque            = interp1(RPM, Torque, SmoothRPM, 'spline');
SmoothPower             = interp1(RPM, Power, SmoothRPM, 'spline');


%%
    
    car.mass                = mass;
    car.NDIM4               = NDIM4;
    car.NDIM6               = NDIM6;
    car.roadCoefficient     = roadCoefficient;
    car.cL                  = cL;
    car.cD                  = cD;
    car.DRS_cL              = DRS_cL;
    car.DRS_cD              = DRS_cD;
    car.rev_cL              = rev_cL;
    car.rev_cD              = rev_cD;
    car.frontArea           = frontArea;
    car.COPdistance         = COPdistance;
    car.COPHeight           = COPHeight;
    car.staticCamberF       = staticCamberF;
    car.staticCamberR       = staticCamberR;
    car.rollCamberF         = rollCamberF;
    car.rollCamberR         = rollCamberR;
    car.pitchCamberR        = pitchCamberR;
    car.pitchCamberF        = pitchCamberF;
    car.cgdistance          = cgdistance;
    car.lateralCGDistance   = lateralCGDistance;
    car.wheelbase           = wheelbase;
    car.TLLTD               = TLLTD;
    car.cgheight            = cgheight;
    car.Ftrack              = Ftrack;
    car.Rtrack              = Rtrack;
    car.AVGtrackwidth       = AVGtrackwidth;
    car.driverCorneringCoP  = driverCorneringCoP;
    car.driverBrakingCoP    = driverBrakingCoP;
    car.driverAcceleratingCoP = driverAcceleratingCoP;
    car.DRS_Enabled         = DRS_Enabled;
    car.RPM                 = RPM;
    car.Torque              = Torque;
    car.Power               = Power;
    car.gears               = gears;
    car.shiftTime           = shiftTime;
    car.redline             = redline;
    car.dt_ratio            = dt_ratio;
    car.primary_reduction   = primary_reduction;
    car.wheel_circum        = wheel_circum;
    car.wheel_dia           = wheel_dia;
    car.brakeBias           = brakeBias;
    car.fixedBrakeBias      = fixedBrakeBias;
    car.SpeedRangeMPS       = SpeedRangeMPS;
    car.SpeedRangeCoarseMPS = SpeedRangeCoarseMPS;
    car.SpeedRangeKPH       = SpeedRangeKPH;
%    car.motorType           = motorType;
    car.Crr                 = Crr;
    car.topSpeed            = topSpeed;
    car.contRegenkW         = contRegenkW;
    
    car.swheel_wheel_rat    = stwheel_wheel_ratio;
    car.maxWangle           = max_wangle;
    car.cg2faxle            = COG2Faxle;
    car.cg2raxle            = COG2Raxle;
    car.MoIx                = Ix;
    car.MoIy                = Iy;
    car.MoIz                = Iz;
    car.tire                = tire;

%% Max Brake Gs and Corresponding Wheel Braking Forces
[BrkG] = brakeG(runPacejka, tire, car, GTol, maxItr);
Brakes.maxBrakeG = max(BrkG.G); %Compute max braking G
maxGindex = find(BrkG.G==Brakes.maxBrakeG); 
Brakes.maxBrakeF = BrkG.BForce(1,maxGindex,:);  %Corresponding braking force per wheel
maxBrakeT = (-car.mass * Brakes.maxBrakeG * g - Brakes.maxBrakeF) * Brakes.Reffective; %Compute approximate torques
Brakes.maxBrakeT = [maxBrakeT(1);maxBrakeT(2);maxBrakeT(3);maxBrakeT(4)]; %Restructure to column vector
maxBias = BrkG.Bias(maxGindex);
Brakes.maxBias = maxBias;

%%
T32CarParamObj = matfile('T32Params.mat');
save('T32CarParamObj');
    