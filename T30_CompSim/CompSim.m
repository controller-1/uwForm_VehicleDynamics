% This competition simulator is a steady state lap sim with points. Dynamic
% event times are calculated, scaled according to historical competition
% data and FSAE rules, and converted to total points.
%
% To run the simulation, modify vehicle parameters of interest in the
% CarParams.m file. Any number of setups may be created and run. 'Setups'
% below must match the number of setups defined in CarParams. Then run
% CompSim.m. Some simulation specific settings may be changed in this file
% such as force calculation and verbose output.
%
% The next intended improvement is to include the CALSPAN TTC4,5 Tire data and to
% model basic transient behaviour focusing on weight transfer effect on grip.
%
% Revisions and credits are at the end of this file
%
% Writen by Daniel Wageman
% Updated 161024
%
% Setup Environment
clear all;
% close all;
% clc;
% Start Elapsed Time Timer
tic

%% Settings
% Plotting and Output Options
FSG                        = 1;
FSAEL                      = 2;
emailData                  = 0;                 % Switch to send email of data to data@uwashingtonfsae.com
plotsOn                    = 1;                 % Switch to plot data
verboseOutputOn            = 1;                 % Switch to turn on Text output in command window
errorOutputOn              = 0;                 % Output simulation % error
debuggingOn                = 1;                 % Special Output Debugging
units                      = 1;                 % 1 for English, 2 for SI
calculateForces            = 0;                 % Calc AutoX forces
outputLoadCases            = 0;                 % Save load cases to XLS sheet
calculateCombinedForce     = 0;                 % Calculate estimated combined loading
% Parameters
setups                     = 1;                 % Number of setups to look at
compName                   = {'FSG', 'FSAEL'};  % Competition names for output
comp2Run                   = [FSG, FSAEL];    % Competitions to run
comp2Plot                  = FSG;
BaseCost                   = [19225,    19225]; % Base Vehicle Cost (NOT including motor) (from FSAEL 2015)

% OUR data for accuracy comparisons from T26 at 2015 FSAEL and T26 at 2015 FSG
compAccelTime              = [4.426,    4.104];
compSkidpadTime            = [5.814,    4.849];
compAutoXTime              = [75.436,   59.074];
compEnduranceTime          = [1474.44,  1432.429]; %FSG From T25, Lincoln extrapolated to make up for DNF
compFuelUsed               = [.761,     .928]; %FSG From T25, Lincoln extrapolated to make up for DNF
% Comp Setup
numEnduroLaps              = [18,       16];             % FSG 2012, FSAEL 2012
autoX2EnduroFactor         = [1,        1];              % Difference between fastest autox and fastest endurance at FSAEL 2012
fastestEnduranceTimeRaw    = [1308.88,  1262.96];       % GFR FSG 2015, Texas A&M FSAEL 2015, Extrapolated from fastest lap time - Scaled to allow for sensitivity analysis
autoXCorrection            = [1,        1];             % Use to correlate actual time to predicted for pure points analysis USE 1.0 FOR RAW TIME PREDICTION

% Event Points @ 2015 FSAEL competition
Scoring(FSG).Accel.Points           = 75;
Scoring(FSG).Accel.FastestTime      = 3.240;
Scoring(FSAEL).Accel.Points         = 100;
Scoring(FSAEL).Accel.FastestTime    = 4.121;

Scoring(FSG).Skidpad.Points         = 75;
Scoring(FSG).Skidpad.FastestTime    = 4.814;
Scoring(FSAEL).Skidpad.Points       = 75;
Scoring(FSAEL).Skidpad.FastestTime  = 5.274;

Scoring(FSG).AutoX.Points           = 100;
Scoring(FSG).AutoX.FastestTime      = 72.05;
Scoring(FSAEL).AutoX.Points         = 125;
Scoring(FSAEL).AutoX.FastestTime    = 91.090; %fastest endurance lap

Scoring(FSG).Enduro.Points          = 325;
Scoring(FSG).Enduro.FastestTime     = fastestEnduranceTimeRaw(1);
Scoring(FSAEL).Enduro.Points        = 275;
Scoring(FSAEL).Enduro.FastestTime   = fastestEnduranceTimeRaw(2);

Scoring(FSG).Economy.Points         = 100;
Scoring(FSG).Economy.FuelMin        = .52;
Scoring(FSG).Economy.FuelMax        = .930;
Scoring(FSG).Economy.EnergyMin      = 4.1;  %% ****BS VALUE FOR FIXING/TESTING****
Scoring(FSAEL).Economy.Points       = 100;
Scoring(FSAEL).Economy.FuelMin      = .510;
Scoring(FSAEL).Economy.FuelMax      = 3.885;
Scoring(FSAEL).Economy.CO2Min       = 5.391; %Min. kg CO2, taken by multiplying CO2 per lap by enduro laps, taking Min
Scoring(FSAEL).Economy.CO2Max       = 15.717; %Max. kg CO2, taken by multiplying CO2 per lap by enduro laps, taking Max
Scoring(FSAEL).Economy.EnergyMin    = 4.1;  %% ****BS VALUE FOR FIXING/TESTING****

Scoring(FSG).Cost.Points            = 45;
Scoring(FSG).Cost.CostMin           = 8000;
Scoring(FSG).Cost.CostMax           = 32000;
Scoring(FSAEL).Cost.Points          = 65;
Scoring(FSAEL).Cost.CostMin         = 7611;
Scoring(FSAEL).Cost.CostMax         = 38120;

wetPadCompensation                  = [1, 1];   % No compensation for 2018, no wetpad. 
                                                %[(Scoring(FSG).Skidpad.FastestTime / Scoring(FSAEL).Skidpad.FastestTime), 1];
Scoring(FSG).CostAnalysisPoints     = 100;
Scoring(FSG).PresentationPoints     = 75;
Scoring(FSG).DesignPoints           = 150;
Scoring(FSAEL).CostAnalysisPoints   = 100;
Scoring(FSAEL).PresentationPoints   = 75;
Scoring(FSAEL).DesignPoints         = 150;

% For theoretical total points, use typical static event finish points
% Based on FSG 2015 scores
Scoring(FSG).typicalCostRatio            = .46;
Scoring(FSG).typicalPresentationRatio    = .68;
Scoring(FSG).typicalDesignRatio          = .8;
Scoring(FSAEL).typicalCostRatio          = .77;
Scoring(FSAEL).typicalPresentationRatio  = .9;
Scoring(FSAEL).typicalDesignRatio        = .96;

%% End of Settings

% Setup Progress Bar
wbar.bar                    = waitbar(0,'Initializing Simulation', 'Name', 'CompSim Progress');

% Make progress bar taller to handle 3 lines of info
set(wbar.bar, 'Position', get(wbar.bar, 'Position') + [0, 0, 0, 10]);

% Add important tracking vars to the progress bar structure
wbar.setups                 = setups;
wbar.comps                  = length(comp2Run);

% Fuel Correlations
% This seems to work pretty well and is quantitatively correct
fuelEnergyDensity           = 36.6;                           % Gasoline energy density in kWh/gal (from Wikipedia)
thermalEfficiency           = .25;                            % Approximate thermal efficiency. Adjust this to correlate
usableEnergyDensity         = fuelEnergyDensity * thermalEfficiency;
fuelConversionFactor        = usableEnergyDensity;               % Variable name change

Points.estCostScore         = ([Scoring(FSG).CostAnalysisPoints, Scoring(FSAEL).CostAnalysisPoints] - [Scoring(FSG).Cost.Points, Scoring(FSAEL).Cost.Points]) .* [Scoring(FSG).typicalCostRatio, Scoring(FSAEL).typicalCostRatio];
Points.estPresentationScore = [Scoring(FSG).PresentationPoints, Scoring(FSAEL).PresentationPoints] .* [Scoring(FSG).typicalPresentationRatio, Scoring(FSAEL).typicalPresentationRatio];
Points.estDesignScore       = [Scoring(FSG).DesignPoints, Scoring(FSAEL).DesignPoints] .* [Scoring(FSG).typicalDesignRatio, Scoring(FSAEL).typicalDesignRatio];

waitbar(0, wbar.bar, 'Running Simulation');

% Build progress bar 'increments' - total of all events
wbar.inc                    = 6 * setups * length(comp2Run);

% Loop through both competitions, all setups
run             = 0;
rStep           = 0;
rStartTime      = 0;
rEndTime        = 0;
for c = comp2Run
    track(c) = Track(c);
    
    % Loop through each setup
    for s = 1 : setups
        % Tracking for progress bar
        run                         = run + 1;
        
        if(debuggingOn); disp(['Run (', num2str(c), ',', num2str(s),') start: ',num2str(toc),' seconds']); end;
        
        % Figure out if we have looped through one setup yet
        if(rEndTime)
            remTime                 = [num2str(round((1 - ((run - 1) / (setups * length(comp2Run)))) * ((rEndTime - rStartTime)*(setups * length(comp2Run))))),' Sec'];
        else
            remTime                 = 'Computing...';
        end
        
        rStartTime                  = toc;

        rStep                       = rStep + 1;
        
        wbar.msg                    = sprintf(['Running ',compName{c}, ' Setup ', num2str(s),' Setup and Params\nEst Time Rem: ',remTime,'\n']);
        wbar.prog                   = (rStep - 1) / wbar.inc;
        
        waitbar(wbar.prog, wbar.bar, wbar.msg)

        % Run CarParameters Function to set up Simulation
        wallTime(c,s).Setup.start   = toc;
        [carSetup(c,s)]             = CarParams(s);
        wallTime(c,s).Setup.end     = toc;
        wallTime(c,s).Setup.t       = wallTime(c,s).Setup.end - wallTime(c,s).Setup.start;
        
        if(debuggingOn); disp(['Setup (', num2str(c), ',', num2str(s),') Wall Time: ', num2str(wallTime(c,s).Setup.t),' seconds']); end;

        % Run Each Event
        % Accel
        rStep                       = rStep + 1;
        wbar.msg                    = sprintf(['Running ',compName{c}, ' Setup ', num2str(s),' Accel\nEst Time Rem: ',remTime,'\n']);
        wbar.prog                   = (rStep - 1) / (wbar.inc);
        
        waitbar(wbar.prog, wbar.bar, wbar.msg)
        
        wallTime(c,s).Accel.start   = toc;
        [Results(c,s).Accel]        = AccelerationEvent(carSetup(c,s), Scoring(c), wbar);
        wallTime(c,s).Accel.end     = toc;
        wallTime(c,s).Accel.t       = wallTime(c,s).Accel.end - wallTime(c,s).Accel.start;
        
        if(debuggingOn); disp(['Accel (', num2str(c), ',', num2str(s),') Wall Time: ', num2str(wallTime(c,s).Accel.t),' seconds']); end;
        
        % Skidpad
        rStep                       = rStep + 1;
        wbar.msg                    = sprintf(['Running ',compName{c}, ' Setup ', num2str(s),' Skidpad\nEst Time Rem: ',remTime,'\n']);
        wbar.prog                   = (rStep - 1) / wbar.inc;
        
        waitbar(wbar.prog, wbar.bar, wbar.msg)
        
        wallTime(c,s).Skid.start    = toc;
        [Results(c,s).Skidpad]      = SkidpadEvent(carSetup(c,s), Scoring(c), wetPadCompensation(c), c);
        wallTime(c,s).Skid.end      = toc;
        wallTime(c,s).Skid.t        = wallTime(c,s).Skid.end - wallTime(c,s).Skid.start;
        
        if(debuggingOn); disp(['Skidpad (', num2str(c), ',', num2str(s),') Wall Time: ', num2str(wallTime(c,s).Skid.t),' seconds']); end;

        % AutoX
        rStep                       = rStep + 1;
        wbar.msg                    = sprintf(['Running ',compName{c}, ' Setup ', num2str(s),' AutoX\nEst Time Rem: ',remTime,'\n']);
        wbar.prog                   = (rStep - 1) / wbar.inc;
        
        waitbar(wbar.prog, wbar.bar, wbar.msg)
        
        wallTime(c,s).AutoX.start   = toc;
        [Results(c,s).AutoX]        = AutoXEvent(carSetup(c,s), Scoring(c), track(c).ATrack, autoXCorrection(c), calculateForces, c, wbar);
        wallTime(c,s).AutoX.end     = toc;
        wallTime(c,s).AutoX.t       = wallTime(c,s).AutoX.end - wallTime(c,s).AutoX.start;
        
        if(debuggingOn); disp(['AutoX (', num2str(c), ',', num2str(s),') Wall Time: ', num2str(wallTime(c,s).AutoX.t),' seconds']); end;

        % Enduro
        rStep                       = rStep + 1;
        wbar.msg                    = sprintf(['Running ',compName{c}, ' Setup ', num2str(s),' Enduro\nEst Time Rem: ',remTime,'\n']);
        wbar.prog                   = (rStep - 1) / wbar.inc;
        
        waitbar(wbar.prog, wbar.bar, wbar.msg)
        
        wallTime(c,s).Enduro.start  = toc;
        [Results(c,s).Enduro]       = EnduranceEvent(carSetup(c,s), Scoring(c), track(c), fuelConversionFactor, numEnduroLaps(c), c, wbar);
        wallTime(c,s).Enduro.end    = toc;
        wallTime(c,s).Enduro.t      = wallTime(c,s).Enduro.end - wallTime(c,s).Enduro.start;
        
        if(debuggingOn); disp(['Enduro (', num2str(c), ',', num2str(s),') Wall Time: ', num2str(wallTime(c,s).Enduro.t),' seconds']); end;

        % Cost
        rStep                       = rStep + 1;
        wbar.msg                    = sprintf(['Running ',compName{c}, ' Setup ', num2str(s),' Cost\nEst Time Rem: ',remTime,'\n']);
        wbar.prog                   = (rStep - 1) / wbar.inc;
        
        waitbar(wbar.prog, wbar.bar, wbar.msg)
        
        wallTime(c,s).Cost.start    = toc;
        [Results(c,s).Cost]         = CostEvent(Scoring(c), BaseCost(c) + carSetup(c,s).otherCost, carSetup(c,s).displacement);
        wallTime(c,s).Cost.end      = toc;
        wallTime(c,s).Cost.t        = wallTime(c,s).Cost.end - wallTime(c,s).Cost.start;
        
        if(debuggingOn); disp(['Cost (', num2str(c), ',', num2str(s),') Wall Time: ', num2str(wallTime(c,s).Cost.t),' seconds']); end;

        % First endtime for est time remaining calc
        rEndTime                    = toc;
    end
end
if(debuggingOn); disp(['Runs Complete at: ',num2str(toc),' seconds']); disp(' '); end;

% Finish waitbar
waitbar(1, wbar.bar, 'Done')

% Error calculations
% NOTE: these are only accurate when comparing actual car to events it attended with the same tire data, parameters, etc
for c = comp2Run
    for s = 1 : setups
        Points.TotalPoints(c,s)        = Results(c,s).Accel.Score + Results(c,s).Skidpad.Score + Results(c,s).AutoX.Score + Results(c,s).Enduro.Score + Results(c,s).Enduro.EfficiencyScore + Results(c,s).Cost.Score;

        Points.accelError(c,s)          = abs((compAccelTime(c)-Results(c,s).Accel.Time)/compAccelTime(c));
        Points.skidpadError(c,s)        = abs((compSkidpadTime(c)-Results(c,s).Skidpad.Time)/compSkidpadTime(c));
        Points.autoXError(c,s)          = abs((compAutoXTime(c)-Results(c,s).AutoX.Time)/compAutoXTime(c));
        Points.enduranceError(c,s)      = abs((compEnduranceTime(c)-Results(c,s).Enduro.Time)/compEnduranceTime(c));
        Points.fuelError(c,s)           = abs((compFuelUsed(c)-Results(c,s).Enduro.FuelBurned)/compFuelUsed(c));
        Points.autoXAvgSpeed(c,s)       = (mean(Results(c,s).AutoX.LoggedSpeed));
        Points.autoXMaxSpeed(c,s)       = (max(Results(c,s).AutoX.LoggedSpeed));
%         Points.maxSpeedIndex(c,s)       = find(carSetup(c,s).TracG.DriveG == 0, 1);
        [~, Points.maxSpeedIndex(c,s)]  = min(carSetup(c,s).TracG.DriveG);
        Points.maxSpeedDrag(c,s)        = carSetup(c,s).car.SpeedRangeMPS(Points.maxSpeedIndex(c,s));
        Points.maxSpeedLatG(c,s)        = carSetup(c,s).LatG.LG(Points.maxSpeedIndex(c,s));
        Points.maxSpeedBrakeG(c,s)      = carSetup(c,s).BrkG.G(Points.maxSpeedIndex(c,s));
    end     
end

% Total points extimate including estimated static events
Points.TotalCompPoints                  = Points.TotalPoints + repmat([Points.estCostScore(:, length(comp2Run)) + Points.estPresentationScore(:, length(comp2Run)) + Points.estDesignScore(:, length(comp2Run))], setups,length(comp2Run))';

% Find point differences between setups per competition
Points.PointDifference                  = diff(Points.TotalPoints,1,2);

% Delete progress bar
delete(wbar.bar);

% Stop Timer
solutionTime                            = toc;
wallTime(1, 1).solutionTime             = solutionTime;

% Output results
save('mat\SimData');

% Evaluate load cases
loadCases();

% Save results in xls file
results(carSetup, Results, Points);

% sendData(emailData);        % Email data to data@uwashingtonfsae.com %Need to reconfigure what data is sent.

% Output results in command window
output(carSetup, Results, Points, wallTime);

% Plot data
if plotsOn; plotData(units, comp2Plot, carSetup, Results, track); end

% % Calculate all tire and member forces
% if calculateCombinedForce
%     [memberForces,tireForces]=ellipseG();
%     save('mat\CombinedForces','memberForces','tireForces')
% end

% Save for diagnostics
save('mat\CompSim')

% Stop run timmer
toc

%% CHANGELOG
% 141026 - Daniel Wageman
% There are 3 things that are important when chosing a house.
% Units, units, and units.
% Depricating opperspeed outside of this function (and maybe G funs) - Use
% SpeedRangeMPS from here on out (and SpeedRangeKPH for plotting)
% General cleanup and commenting

%% TODO
% ** Now **
% Better diag time tracking
% Data Logging During Event. Acceleration, Speed, RPM
% Fix shift point calc
% Brake Temp
% Clean up
% Comment
%
% ** Later **
% Fix Data emailing
% Pacejka tire model inclusion
% Clamped or BSpline based corners
% Trail Braking and Corner Exit based on traction circle [Hint iterate back to entry speed using MIN radius of spline, calc braking force remaining after required lateral force of cornering is applied]
% Dynamic Camber
% Compliance Camber
% Yaw Rate
% Fix BaseTrack vs front and rear track remove?
% New FSG Track
%
%% Intellectual Credit for this Simulation belongs to:
% Gilbert Gede          Original Creator
% Brandon Cattanach     Dedicated Accel and Skidpad Events
% Daniel Wageman        Vehicle Dynamics, Validation, functionalization,
%                       Points Analysis, Cost, and 'true' Fuel Incorporation
% Josh Pu'u             Vehicle Dynamics, Dynamic Camber and Compliance
% Mikk Kaschko          Bezier Curves and UnitVectorFinder
% Jeron Moore           Dynamic Camber and Track Point Descriptions
% Eben Kiehl            Suspension forces and validations
%
%% Revision History
% V1 - ~2008 Gilbert Gede
% V2 - ~2010 Branden Cattanach
% V3 - 2011 Daniel Wageman - Created function calls, fixed aero sign errors, completed points analysis
% V3.0.1 - 10/01/11 - Daniel Wageman - Added data sending to data@uwashingtonfsae.com, fix other minor errors
% V3.0.2 - 10/26/11 - Daniel Wageman - Added Jeron's 'bad' racing line track to do sensitivity analysis on racing line
% V3.1.0 - 10/26/11 - Daniel Wageman - Added gear tracking, including shift count and current gear at all points. Needs slight fix
% V3.2.0 - 10/30/11 - Daniel Wageman - Made main .m a function, work on overloading CarParams so that it can be run from there. Course deff now contained in CarParams and passed to AutoX. Track width now drives Load Transfer and change in effective track radius
% V3.2.1 - 10/31/11 - Daniel Wageman - Created Lateral Weight distribution handling. Incorporated into autox with + and - radii indicating left or right hand turns
% V3.2.2 - 11/01/11 - Daniel Wageman - Re-wrote slip angle sweep iteration using log bisection method. 75% faster solutions now
% V3.3.0 - 11/01/11 - Daniel Wageman - Re mapped track input to accept purely points as a stepping stone to mapping corners as weighted splines in the near future. Both x,y endpoints of the straights are required as well as the simple center (symetric) apex of the corner.
% V3.3.1 - 11/06/11 - Daniel Wageman - Fixed track point issues
% V3.4.0 - 11/08/11 - Daniel Wageman - Fixed track point issues, connected radii
% V3.4.2 - 11/10/11 - Daniel Wageman - Created more function calls
% V3.5.0 - 11/11/11 - Daniel Wageman - More functions.... BFit work
% V3.5.1 - 11/12/11 - Daniel Wageman - Work on BFit to get radius of curvature
% V3.5.2 - 11/13/11 - Daniel Wageman - Added full CG calculations based on components
% V3.5.5 - 11/16/11 - Daniel Wageman - Tons of work on splines. Added a lot of data return from accel event. Fixed some plotting
% V3.5.6-V3.5.9 - 11/28/11 - Daniel Wageman - Several major revisions. Validated aero force, fixed inclination angle issues, correct response from Lat G and Drive G from Lat CG location calculations. Updated Accel and Skidpad to incorporate said changes. Added error calculations.
% V3.5.10-V3.5.11 - 11/30/11 - Daniel Wageman - Pulled min times out of functions. Shifting now fixed (accurate)
% V3.6.6 - 05/31/12 - Daniel Wageman - Missing lots of revision comments, many small fixes
% V3.6.7 - 06/02/12 - Daniel Wageman - Parameter Updates
% V3.6.8 - 06/04/12 - Daniel Wageman - Added accel plots, added extraCost parameter to carparams.m
% V3.6.9 - 06/12/12 - Daniel Wageman - Added FSG Points and calculations. Found error in # of laps in endurance NEED TO CHANGE, currently in the middle of sensitivity analysis
% V3.6.10 - 06/15/12 - Daniel Wageman - Changed all interp1 functions to use cubic instead of spline, saved 6 seconds of solution time with no noticable loss of accuracy. Added preliminary Lincoln map
% V3.6.11 - 06/15/12 - Daniel Wageman - Found big aero drag error, changed to realistic cD, cL numbers for T23 car. Also found error in Accel event length. Fixed single cylinder torque numbers
% V3.6.12 - 06/16/12 - Daniel Wageman - Fixed left hand lateral G issue, fixed errors in for/aft calculated cg location and fixed error with weight transfer due to COP hight and aero drag
% V3.6.13 - 06/17/12 - Daniel Wageman - Misc small fixes and plot changes
% V3.6.14 - 06/18/12 - Daniel Wageman - Added SI Plots
% V3.6.15 - 06/22/12 - Daniel Wageman - Basic attempts at adding Lincoln enduro course. Also attempted correlation
% V3.6.16 - 06/26/12 - Daniel Wageman - Many small fixes. Re-wrote fuel used based on empirical data and direct conversions. Fixed acceleration power misconception
% V3.6.17 - 08/04/12 - Daniel Wageman - Small fixes and FSG specific changes
% V3.7.0 - 09/07/12 - Daniel Wageman - DRS solution code incorporated. Simplistic binary system, DRS only active on accel
% V3.7.1 - 10/22/12 - Daniel Wageman - Clean-up of old code/comments, cleaned up output, fixed masses in car params, fix of comp points, added total comp points (with static events), included correlation between autox and enduro times, moved enduro lap count to main function
% V3.7.2 - 10/24/12 - Daniel Wageman - Fixed number of gears, created more generic launch conditions
% V3.7.3 - 10/24/12 - Daniel Wageman - Massive rewrite of G functions, total solve time reduced by 1 full order of magnitude. Convergence tolerance is now variable.
% V3.7.4 - 10/25/12 - Daniel Wageman - Fixed Shift Point representation and sensitivity, moved track deff outside of CarParams. Using updated expand_mrandim.
% V3.7.5 - 10/25/12 - Daniel Wageman - Added regenerative breaking energy calculations on rear wheel braking conditions.
% V4.0.0 - 10/26/12 - Daniel Wageman - Massive rewrite of all G force functions. Use built in SA peak finder in LatG. Added tracking of forces contact patch forces.
% V4.0.1 - 10/26/12 - Daniel Wageman - Single tractiveG function (DRS_Enabled is read internally). Drive/AG math pulled into tractiveG.
% V4.0.2 - 10/26/12 - Daniel Wageman - Setup flexible now. Any number of setups can now be run and output and plotted. Fixed most plot and legend entries. Still missing AutoX trace and some shifting in plots.
% V4.0.3 - 10/28/12 - Daniel Wageman - Added and fixed all force calcs and plots. Most other plots fixed and finished. Shift points of accel and autox still not working.
% V4.0.4 - 10/28/12 - Daniel Wageman - Fixed tractive G aero pitching error. Added aero tire normal force contribution. Lots of clean-up and general check over.
% V4.0.5 - 11/02/12 - Daniel Wageman - Fixed Aero COP height issue. Now finds and uses COP above CG. Fixed tractive forces to reflect torque limited condition. Removed AG from all relevant functions, use DrvG instead.
% V4.0.6 - 11/06/12 - Daniel Wageman - Re-Fixed Aero COP height. Use total COP to find pitching moment. Tweaked Torque plots.
% V4.0.7 - 11/09/12 - Daniel Wageman - Reverse braking force, other small tweaks.
% V4.0.8 - 11/10/12 - Daniel Wageman - Fixes small errors, checks for open EventResults file before writing to it.
% V4.0.9 - 11/12/12 - Daniel Wageman - CarParams now overloaded function. Added many comments in CarParams. Top speed now fixed to remedy some DT ratio sweep issues.
% V4.0.10 - 11/17/12 - Daniel Wageman - Fixed reverse braking, member forces and ellipseG now working properly. Small optimizations.
% V4.0.11 - 11/17/12 - Daniel Wageman - Addition and trials of electric motor calculations.
% V4.0.12-13 - 11/19/12 - Daniel Wageman - Fixed lateralG issues, added many comments. Checked over other forces and tweaked minor parameters.
% V4.0.14 - 11/19/12 - Daniel Wageman - Fixed errors in ellipseG and added some parts of a reverse solver for checking the accuracy of the ellipsoidal estimation.
% V4.1.0 - 11/23/12 - Daniel Wageman - Now loops through both competitions every time its run. Added some comments.
% V4.1.1 - 11/23/12 - Daniel Wageman - Many tweaks, check overs, and comments.
% V4.1.2 - 11/27/12 - Daniel Wageman - New shift counting algorithm (almost correct). Allows for single competitions (either) to be run independently or in any combination. Scaled wetpad and autox based on fastest times from each competition until proper tracks are defined. Some small band aids and comments applied.
% V4.1.3-4b - 11/30/12 - Daniel Wageman - AutoX comments and check over. Complete re-write of energy usage and fuel usage. Draft DRS activation tracking and attempt at correcting shift tracker. Re-arranged parameters in CarParams to allow for better setup differentiation.
% V4.1.5b - 12/01/12 - Daniel Wageman - Many simplifications and bundling of parameters. General function clean-up of unused things.
% V4.1.6b - 12/01/12 - Daniel Wageman - More parameter bundling and debugging.
% V4.1.7b - 12/02/12 - Daniel Wageman - Final parameter bundling. Everything seems to be working.
% V4.1.8b - 12/29/12 - Daniel Wageman - eTrain experimentation and fixing of small bugs in the motor calculations.
% V4.1.9-11 - 130601 - Daniel Wageman - Plot and figure fixes
% V4.1.12 - 130701 - Daniel Wageman - Plot and figure fixes, fixed issue with autoX data not lining up on distance - did not take dist covered durring shift into account. Other small fixes and tweaks from comp as well.
% V4.1.13 - 130704 - Daniel Wageman - Fixed issue of overshooting end of straights while in shift loop. Other small fixes.
% V4.1.14 - 130705 - Daniel Wageman - Added switches for plots, disabled regen plot with ICE. Added front and rear brake force + bias.
% V4.2.1 - 131002 - Daniel Wageman - Fixed Economy calls for 2014 rules changes. Other small tweaks for comparing 10" tires using simplified coefficients.
% V4.2.2 - 131012 - Daniel Wageman - Scaled best enduro time to allow for 'better' sensitivity analysis.
% V4.2.3 - 131015 - Daniel Wageman - Started getting everything tweaked for T25 car. Re-tooled lateralG to use a while loop and break on convergence error.
% V4.2.4rc - 131022 - Daniel Wageman - Fixed various small issues. Added drag force to brakeG calc. Fixed Pacejka lookup broken in ML 2013. Renamed some variables and for loops for easy of understanding. Changed coarse speed range to 5mph increments.
% V4.2.5rc - 131024 - Daniel Wageman - Fixed little issues and sped up some iterative solutions in the G calcs. Improved Pacejka equations - still not quite accurate. Suspect issue with fuel calc now, predicts very low usage.
% V4.2.6rc - 131025 - Daniel Wageman - Fixed MANY issues all around. Added pitch and roll camber change. Simulation no longer runs out of gears during the accel even (if too short of gearing is selected). Issue with negative interpolated corner speed fixed NOT WORKING FULLY.
% V4.2.7rc - 131031 - Daniel Wageman - Implemented early Pacjeka based tire model. Function behaves very similar to original mrandmin.
% V4.3.0rc - 131103 - Daniel Wageman - Minor issue fixed
% V5.0.0b - 131103 - Daniel Wageman - Major revision, all calculations in metric now - Pacejka data validation vs MRA.
% V5.0.1b - 131111 - Daniel Wageman - Tweaks of autox event to debug and prevent infinite looping. Fully running on Pac tire data but many event still off. Some debugging still needed on AutoX.
% V5.0.2b - 131112 - Daniel Wageman - Lots of rework, non functional
% V5.0.3 - 5.0.4 - 131225 - Daniel Wageman - Still reworking, non functional
% V5.0.5 - 140330 - Daniel Wageman - First functional version of rework. Load and Save routines have been eliminated in all but output and plots. Structures have extensively replaced cell arrays, and the 2014 Lincoln autoX and enduro tracks have been defined and added along with 2013 FSG. The track deff and ploting has been redefined and repaired.
% V5.1.0 + - Daniel Wageman - CompSim moved to SVN at https://uwashingtonfsae.svn.cloudforge.com/compsim
% 151011 - Daniel Wageman - Fixed issue with max speed solution if car is setup such that it is not gearing limited before simulation top speed.
% 161013 - Daniel Wageman - Points changes merged from Jon Anderson's setup from T27
% 161024 - Daniel Wageman - eCar functions fixed