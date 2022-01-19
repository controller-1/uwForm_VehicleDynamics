% The function sets up all major car parameters based on a set of inputs
% defined in the first cell. If not passed a setup number, it assumes to
% have been run directly from this function and instead executes CompSim.m
%
% This is the only function that should be edited in normal use.
%
% Writen By Daniel Wageman
% Updated 161024
%
function [OUTPUTS] = CarParams(varargin)%SpeedRangeMPH
% Overload function. If no variable passed in (ie this file is executed directly) execution will pass to the parent function CompSim. This just makes it easier to run CompSim without having to switch files after changes.
if nargin == 0
    CompSim()
else
    % Save setup if it was passed in
    Setup = varargin{1};
    
    %% Car Settings (Track Settings Below)
    % Set this variable to the values you want to sweep through and set the variable below to var=parameter(Setup).
    parameter = 0;                      % Dummy to support return
    %     parameter =                     [.95,.975,1,1.025,1.05];
    %     parameter =                     [.9, .95, 1, 1.05, 1.1];
    %     parameter =                     [.8,.9,1,1.1,1.2];
    %     parameter =                     [.5,.75,1,1.25,1.5];
    %     parameter =                     -[.5,.75,1,1.2,1.3];
    %     parameter =                     [.8,.9,1,1.25,1.5];
    %     parameter =                     [.05,.075,.1,.2,.3];
    %     parameter =                     [0, 1];
    %     parameter =                     [.9,  1,  1.1];
    %     parameter =                     [.1, .3, .5, .7, .9];
    %     parameter =                     [.95,  1,  1.05];
    %     parameter =                     [-1,-.75,-.5,-.25,0,.25,.5,.75,1];
    %     trackW =                     [46,47,48,49,50,51,52,53,54,55,56];
    %     parameter =                       [32,34];
    tireCoP                         = 1;  % Approx grip of 10" tires compared to the 13"s.
    g                               = 9.81;
    namePrefix                      = ('');
    
    % Load MRA File for Legacy Support
    load ('tires\Hoosier 20.5x7.0-13_7rim.mat');% Load tire data (only TTC rnd 3 data is supported so far).
    % NEW Pacejka Format
    % pacejkaFile                   = 'tires\R5_Hoosier 6.0-18.0-10 P12 LC0 Rim7.TIR';
    pacejkaFile                     = 'tires\LC0 Manual Coefficients with Mz.TIR';
    %     pacejkaFile = {'tires\HoosierR25B.tir'};
    tire                            = ImportTireData(pacejkaFile);
    
    %     Name =                          names{Setup};             % Name to use for output, etc
    %     Name =                          [namePrefix,' ',num2str(parameter(Setup))];             % Name to use for output, etc
    Name = ('T30_Initial');                        % Name to use for output, etc
    
    displacement                    = 450;                      % Engine displacement (CC)
    otherCost                       = 0;                        % Any extra cost due to extra features
    motorType                       = 2;                        % Electric or combustion motor. 1 for combustion, 2 for electric
    shiftTime                       = .1;                       % Est shift time (sec)
    wheel_dia                       = .445;                     % Wheel diameter (m)
    useCalculatedMass               = 0;                        % If 1: Calculate mass from individual components. If 0, use forced mass properties.
    
    % Suspension Parameters
    runPacejka                      = 1;                        % Run Pacjeka or MRA tire model
    TLLTD                           = .50;                    % Total lateral load transfer distribution
    wheelbase                       = 1.562;                   % Vehicle wheelbase (m)
    Ftrack                          = 1.2446;                     % Front Track (m)
    Rtrack                          = 1.1938;                     % Rear Track (m)
    pitchCamberF                    = 1.32;                      % Front Pitch Camber Sensitivity
    pitchCamberR                    = 1.5;                      % Rear Pitch Camber Sensitivity
    rollCamberF                     = .662;                      % Front Roll Camber Sensitivity
    rollCamberR                     = .532;                      % Rear Roll Camber Sensitivity
    staticCamberF                   = -1.9;                     % Static Front Camber
    staticCamberR                   = -1.5;                       % Static Rear Camber
    wheelMoI                        = 591.83;                   % Wheel MoI in lb*in^2 (10") from SW
    % wheelMoI                      = 699.5;                    % Wheel MoI in lb*in^2 (13") from SW
    brakeBias                       = .72;
    fixedBrakeBias                  = 1;
    
    % Aero Paramaters
    COPdistance                     = .25;                       % Aero Center of Pressure (percent on front)
    COPHeight                       = .508;                      % Aero Center of Pressure height (m)
    
    % Preliminary method for allowing trail braking and exit acceleration
    % until combined loading calculations are implemented. This coefficient
    % is the % of the min corner (apex) speed allowed at corner entry and
    % exit.
    entryOverSpeed                  = 1.0;
    
    if(motorType == 1)
        DRS_Enabled                 = 0;                          % Turn DRS evaluation on and off
        % Team 25 Predicted Numbers
        cD                          = 1.44;                      % High coeff of BrkG.drag
        cL                          = -3.05;                      % High coeff of lift
        DRS_cD                      = 1.2675;                       % Actually CDA - Aero coeff of BrkG.drag when DRS is enabled
        DRS_cL                      = -2.17;                      % Actually CLA - Aero coeff of lift when DRS is enabled
        rev_cD                      = 10/6.5;                     % Actually CDA - Aero coeff of BrkG.drag when going backward
        rev_cL                      = 3.75/6.5;                   % Actually CLA - Aero coeff of lift when going backward
        frontArea                   = 1;                        % Aero frontal area for BrkG.drag and lift calculation. Usually normalized to 1 by wind tunnel(ft^2)
        % Forced mass properties
        if useCalculatedMass == 1
            % Individual Component Masses format: [mass,CGx,CGy,CGz]
            masses                  = xlsread('excel\mass1.xlsx',1,'B2:E50');
        else
            cgdistance              = .49;                      % Mass percent on front
            lateralCGDistance       = .5;                       % Mass percent on left side of car
            cgheight                = 0.285;                    % CG height (m)
            mass                    = 240;                    % Total vehicle mass + driver (kg)
        end
        primary_reduction           = 61/23;                              % Primary engine reduction from crankshaft to input shaft.
        ratios                      = [29/12,	26/15,	21/16,  21/20, 21/25      % WR Gearing 
                                       35/14,	30/15,	31/19,	28/21, 23/21];    % YFZ Gearing 
        gears                       = ratios(2,:);
        % gears =                   [2.417,1.733,1.313,1.05,0.84,.5];   % YZ Gearing
        drivetrain_sprocket         = 32;                   % Teeth on rear sprocket
        engine_sprocket             = 13;                                 % Teeth on engine sprocket
        redline                     = 12000;                              % Engine Redline
        torqueCorrectionFactor      = .9;                                 % Correction factor for dyno correlation
        % Measured torque and RPM points
        % Stock Engine
        % engineParams =              [0      1000	2000	3000	4000	5000	6000	6200	6450	6650	6820	7180	7720	7970	8390	8710	8892	9080	9350	10000	11000	12000
        %                              0.0	  9.5   23.0	29.8	33.9	38.0	43.4	44.7	46.1	46.8	46.8	46.1	44.7	43.4	42.0	40.7	39.3	38.0	36.6	33.9	31.2	27.1];
        % Shitty T26 Germany Torque
        % RPM                = [0     1000	2000	3000	4000	5000	6000	6500	7000	7500	8000	8500	9000	9500	10000	10500];
        % Torque             = [0 6.5	15.8	20.4	23.3	27.1	29.4	29.9	31.4	32.1	32.0	30.6	31.3	31.6	31.0	30.5];
        % Big Bore Torque
        % RPM                = [0     1000	2000	3000	4000	5000	6000	6500	7000	7500	8000	8500	9000	9500	10000	10500];
        % Torque             = [0.0	9.5     23.0	29.8	33.9	39.5	44.4	47.0	46.7	46.8	47.0	47.3	47.5	46.8	45.4	43.2];
        % RPM                =  [0     1000	2000	3000	4000	5000	6000	6500	7000	7500	8000	8500	9000	9500	10000	10500];
        % Torque             =  [0 6.5	15.8	20.4	23.3	27.1	29.4	29.9	31.4	32.1	32.0	30.6	31.3	31.6	31.0	30.5];
        RPM                = [0     1000        2000    3000    4000    5000    6000    6500    7000    7500    8000    8500    9000    9500    10000   10500];
	    Torque             = [0.0       9.5     23.0    29.8    33.9    39.5    44.4    47.0    46.7    46.8    47.0    47.3    47.5    46.8    45.4    43.2];
        Torque             = torqueCorrectionFactor * Torque;
        Power              = (Torque * (2 * pi()) .* RPM / 60) / 1000;
        
    elseif(motorType == 2)
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
            masses                  = xlsread('excel\mass2.xlsx',1,'B2:E50');
        else
            cgdistance              = .4056;                      % Mass percent on front
            lateralCGDistance       = .5;                       % Mass percent on left side of car
            cgheight                = 0.2728;                     % CG height (m)
            mass                    = 241.325;                  % Total vehicle mass + driver (kg)
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
    else
        warning('Incorrect Motor Type Selected');
    end
    
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
    
    %     topSpeed=80;                          % Max simulation speed KPH
    % Top Speed in KPH!!! NOT MPS!!
%     if(length(gears) == 1)
%         topSpeed = 60 * (redline / (primary_reduction * dt_ratio * gears(end))) * wheel_circum / 1000;
%     else
%         topSpeed = 60 * (redline / (primary_reduction * dt_ratio * gears(end-1))) * wheel_circum / 1000;
%     end

%% added 10/7/2018 by PJ, recalculation for top speed based on losses
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
    
    
    
    
    %% Misc Functions
    trackChange = (baseTrack - ((Ftrack + Rtrack) / 2)) / 2; %Calculated parameter to pass to AutoX (is change in width from what the course was programmed with)
    
    % Manipulate element masses to get CG and other mass properties
    if useCalculatedMass == 1
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
    end
    
    % Setup speed ranges IN MPS
%     operSpeedCoarse     = 0 : 5 : topSpeed;
%     operSpeedCoarse(1)  = 0.1;
%     operSpeedFine       = 1 : 1 : topSpeed;
%     operSpeedCoarse     = linspace(0, topSpeed, Ncoarse);
    % Maybe done need this anymore?
%     operSpeedCoarse(1)  = 0.1;
%     operSpeedFine       = linspace(0, topSpeed, Nfine);

    Ncoarse = 20;
    Nfine = 100;
    
    % Unit conversions
    SpeedRangeCoarseMPS     = linspace(0, topSpeed, Ncoarse);
    SpeedRangeMPS           = linspace(0, topSpeed, Nfine);
    SpeedRangeKPH           = SpeedRangeMPS * (3600 / 1000);          % Convert kph to m/s

    SmoothRPM               = linspace(0, redline, 150);
    SmoothTorque            = interp1(RPM, Torque, SmoothRPM, 'spline');
    SmoothPower             = interp1(RPM, Power, SmoothRPM, 'spline');
    
    % Save it all...
    %     save ('mat\setup')
    
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
    car.motorType           = motorType;
    car.Crr                 = Crr;
    car.topSpeed            = topSpeed;
    
    car.contRegenkW         = contRegenkW;

    
    %% Lateral G's
    
    % Call MRA code to find maximum potential lateral gs
    [LatG] = lateralG(runPacejka, tire, car, GTol,  maxItr);
    
    %% Braking G's
    
    % Call MRA code to find maximum potential braking gs
    [BrkG] = brakeG(runPacejka, tire, car, GTol, maxItr);
    
    % Save forces for suspension work
    %     save('mat\BrkG.BrakeForces','BrkG.AeroF','BrkG.ReverseBrakeG','BrkG.ReverseBrakeForce','BrkG.ReverseAeroF');
    
    %% Max Tractive Force
    
    % Call MRA code to find maximum potential tractive gs
    [TracG] = tractiveG(runPacejka, tire, car, GTol, maxItr);
    
    %% Interpolations and Manipulations
    % Interpolate rough data points to fine data points IN MPS
    LatG.LG     = tireCoP * interp1(SpeedRangeCoarseMPS, LatG.LG, SpeedRangeMPS, 'spline');
    LatG.RG     = tireCoP * interp1(SpeedRangeCoarseMPS, LatG.RG, SpeedRangeMPS, 'spline');
    BrkG.G      = tireCoP * interp1(SpeedRangeCoarseMPS, BrkG.G , SpeedRangeMPS, 'spline');
    
    % Interpolate forces
    % Loop through each axis - x,y,z
    for Axis=1:3
        % Loop through each tire - LF,RF,LR,RR
        for t=1:4
            LLatF(Axis, :, t)     = interp1(SpeedRangeCoarseMPS, LatG.LForce(Axis,:,t), SpeedRangeMPS, 'spline');     % Left lateral force
            RLatF(Axis, :, t)     = interp1(SpeedRangeCoarseMPS, LatG.RForce(Axis,:,t), SpeedRangeMPS, 'spline');     % Right lateral force
            BrakeF(Axis, :, t)    = interp1(SpeedRangeCoarseMPS, BrkG.BForce(Axis,:,t), SpeedRangeMPS, 'spline');       % Braking force
            TractiveF(Axis, :, t) = interp1(SpeedRangeCoarseMPS, TracG.Force(Axis,:,t), SpeedRangeMPS, 'spline'); % Accel force
        end
    end
    
    % Calculated max regenerated power through rear wheels for eCar
    BrkG.regenPower     = interp1(SpeedRangeCoarseMPS, BrkG.regenPower, SpeedRangeMPS, 'spline');
    
    Loads.LLatF         = LLatF;
    Loads.RLatF         = RLatF;
    Loads.BrakeF        = BrakeF;
    Loads.TractiveF     = TractiveF;
    
    % CHANGE THESE IN MOMA FUNCTION
%     TracG.speedInGear       = TracG.speedInGear * 1000 / 3600;      % Convert kph to m/s
%     TracG.minSpeedInGear    = TracG.minSpeedInGear * 1000 / 3600;   % Convert kph to m/s
    
    % Save all parameters to output structure
    OUTPUTS.Name                = Name;
    OUTPUTS.Loads               = Loads;
    OUTPUTS.Engine.Torque       = SmoothTorque;
    OUTPUTS.Engine.Power        = SmoothPower;
    OUTPUTS.Engine.RPM          = SmoothRPM;
    OUTPUTS.drag                = BrkG.drag;
    OUTPUTS.displacement        = displacement;
    OUTPUTS.trackChange         = trackChange;
%     OUTPUTS.SpeedRangeMPS       = SpeedRangeMPS;
%     OUTPUTS.SpeedRangeKPH       = SpeedRangeKPH;
    OUTPUTS.otherCost           = otherCost;
    OUTPUTS.topSpeed            = topSpeed;
    OUTPUTS.mass                = mass;
    OUTPUTS.cgheight            = cgheight;
    OUTPUTS.motorType           = motorType;
    OUTPUTS.TracG               = TracG;
    OUTPUTS.BrkG                = BrkG;
    OUTPUTS.LatG                = LatG;
    OUTPUTS.parameter           = parameter;
    OUTPUTS.staticCamberF       = staticCamberF;
    OUTPUTS.staticCamberR       = staticCamberR;
    OUTPUTS.cgdistance          = cgdistance;
    OUTPUTS.lateralCGDistance   = lateralCGDistance;
    OUTPUTS.car                 = car;
    OUTPUTS.tire                = tire;
    OUTPUTS.Brakes              = Brakes;
    OUTPUTS.entryOverSpeed      = entryOverSpeed;
    
end

% Save for diagnostics
% save('mat\CarParams')

%% CHANGELOG
% 141015 - Daniel Wageman
% Started getting brake temp calcs in. Heat up and convect heat while
% braking, need to add cooling durring the rest
%
% 141019 - Daniel Wageman
% Replaced interp1 'pchip' with 'spline' for speed
%
% 141026 - Daniel Wageman
% There are 3 things that are important when chosing a house.
% Units, units, and units.
% Depricating opperspeed outside of this function (and maybe G funs) - Use
% SpeedRangeMPS from here on out (and SpeedRangeKPH for plotting)
%
% 151011 - Daniel Wageman
% Removed T25 aero cop from base setup.
%
% 161013 - Daniel Wageman
% Changes merged from Jon Anderson's setup from T27 - though torque seems
% definitely incorrect. Later re-merged my old curves from ~T25
%
% 161024 - Daniel Wageman
% Fixed broken eCar and regen functionality
%
% Peter Link
% Updated Ecar Etrain Parameters - changed torque numbers to match Emrax
% 188 motors
