% This function calculates the tractive capacity of the car. It is passed a
% G force convergence tolerance and iteration limit and returns TractiveG,
% DriveG, avgSR, TractiveForce, driveForce_noAero, AccelPower, DragPower,
% ShiftPoints, speedInGear, minSpeedInGear, speedDiff, and gear. It loads
% 'Setup' from CarParams which must be run first.
%
% Writen by Daniel Wageman
% Updated 161024
%
function [Output]=tractiveG(varargin)
if nargin == 0
    CompSim()
else
    runPac          = varargin{1};
    tire            = varargin{2};
    car             = varargin{3};
    gTol            = varargin{4};
    maxIterations   = varargin{5};
    
    g = 9.81;
    
    % Define IA in terms of camber
    % NOTE! From RCVD, IA is + when the top of the tire leans to the right when viewed from behind
    car.cRIA        = -1;
    car.cLIA        = 1;
    leftRearIA      = car.cLIA * car.staticCamberR;
    rightRearIA     = car.cRIA * car.staticCamberR;
    
    % Static tire forces before load transfer and aero
    FL_static       = car.mass * g * (car.cgdistance * car.lateralCGDistance);
    FR_static       = car.mass * g * (car.cgdistance * (1 - car.lateralCGDistance));
    RL_static       = car.mass * g * (1 - car.cgdistance) * car.lateralCGDistance;
    RR_static       = car.mass * g * (1 - car.cgdistance) * (1 - car.lateralCGDistance);
    
    % Preallocate matrices for speed
    % REMOVED - Ended up slowing things down
    
    % Positive SR for acceleration case. Only sign matters
    DummySR         = .1;
    
    % Find realistic starting place
    if runPac
        FResult     = Pacejka(tire, 'px', (-car.mass * g / 4), 0, DummySR, rightRearIA, car.roadCoefficient, tire.LONGVL, 0);
    else
        FResult     = expand_mrandim([106,2], car.NDIM6, [-car.mass * g / 4, 0, rightRearIA, DummySR, car.roadCoefficient]); % Call to MRA Non-dimensional Tire Model
    end
    FPeak           = FResult(2);
    startG          = .9 * abs(2 * FPeak / (car.mass * g));
    
    % Set first point to start
    TractiveG_Old   = startG;
    
    i               = 0;
    keepGoing       = 1;
    while ((keepGoing) && (i < length(car.SpeedRangeCoarseMPS)))
        i               = i + 1;
        speed           = car.SpeedRangeCoarseMPS(i);
        
        % Deal with aero and DRS These are MPS
        if car.DRS_Enabled
            lift        = aeroForce(car.DRS_cL, car.frontArea, speed);
            drag        = aeroForce(car.DRS_cD, car.frontArea, speed);
        else
            lift        = aeroForce(car.cL, car.frontArea, speed);
            drag        = aeroForce(car.cD, car.frontArea, speed);
        end
        FAero           = (-(lift * car.COPdistance) - drag * car.COPHeight / car.wheelbase);
        RAero           = (-(lift * (1 - car.COPdistance)) + drag * car.COPHeight / car.wheelbase);
        
        n               = 0;
        keepSearching   = 1;
        while ((n < maxIterations) && keepSearching)
            n               = n + 1;
            
            LongWT(i)       = car.mass * car.cgheight * (TractiveG_Old * g) / car.wheelbase;
            
            FLZCurrent(i)   = FL_static - (LongWT(i) / 2) + (FAero / 2);
            FRZCurrent(i)   = FR_static - (LongWT(i) / 2) + (FAero / 2);
            RLZCurrent(i)   = RL_static + (LongWT(i) / 2) + (RAero / 2);
            RRZCurrent(i)   = RR_static + (LongWT(i) / 2) + (RAero / 2);
            
            fTest           = (FLZCurrent(i) + FRZCurrent(i) + RLZCurrent(i) + RRZCurrent(i)) - (FL_static + FR_static + RL_static + RR_static + FAero + RAero);
            
            if (abs(fTest) > .001)
                warning('car.mass balance off - check equations')
            end
            
            rightRearIA_pitch   = rightRearIA - car.cRIA * car.pitchCamberR * TractiveG_Old;
            leftRearIA_pitch    = leftRearIA - car.cLIA * car.pitchCamberR * TractiveG_Old;
            
            if runPac
                %                 RRresult    = Pacejka(tire, 'px', -RRZCurrent(i), 0, DummySR, rightRearIA_pitch, car.roadCoefficient, speed, 0);
                %                 RLresult    = Pacejka(tire, 'px', -RLZCurrent(i), 0, DummySR, leftRearIA_pitch, car.roadCoefficient, speed, 0);
                RRresult    = Pacejka(tire, 'px', -RRZCurrent(i), 0, DummySR, rightRearIA_pitch, car.roadCoefficient, tire.LONGVL, 0);
                RLresult    = Pacejka(tire, 'px', -RLZCurrent(i), 0, DummySR, leftRearIA_pitch, car.roadCoefficient, tire.LONGVL, 0);
            else
                RRresult    = expand_mrandim ([106,2], car.NDIM6, [-RRZCurrent(i), 0, rightRearIA_pitch, DummySR, car.roadCoefficient]);
                RLresult    = expand_mrandim ([106,2], car.NDIM6, [-RLZCurrent(i), 0, leftRearIA_pitch, DummySR, car.roadCoefficient]);
            end
            
            RRSR            = RRresult(1);
            RLSR            = RLresult(1);
            avgSR(i)        = (RRSR + RLSR) / 2;
            RRx             = RRresult(2);
            RLx             = RLresult(2);
            
            TractiveG_New   = (RRx + RLx) / (car.mass * g);
            difference      = abs(TractiveG_Old - TractiveG_New);
            TractiveG_Old   = TractiveG_New;
            
            if (difference < gTol)
                itr(i)          = n;
                TractiveG(i)    = TractiveG_New;
                keepSearching   = 0;
            end
            
            if n == maxIterations
                warning('Max tractiveG Iterations Reached! Solution may be incorrect. Try changing maxIttr or check setup.')
                keepGoing       = 0;
            end
            
        end
        
        TractiveForce(:,i,1)    = [0;0;((car.mass * g) / 2) - RLZCurrent(i)];     % FL
        TractiveForce(:,i,2)    = [0;0;((car.mass * g) / 2) - RRZCurrent(i)];     % FR
        TractiveForce(:,i,3)    = [RLx;0;RLZCurrent(i)];            % RL
        TractiveForce(:,i,4)    = [RRx;0;RRZCurrent(i)];            % RR
        
    end
    
    TractiveG       = TractiveG .* car.driverAcceleratingCoP;
    TractiveG       = interp1(car.SpeedRangeCoarseMPS, TractiveG, car.SpeedRangeMPS, 'spline');
    
    GearReduct      = car.gears .* car.primary_reduction * car.dt_ratio;
    
    %     % shift Points
    %     ShiftPoints     = 0;
    %     for i = 1 : (length(car.gears) - 1)
    %         for j = 1000 : 50 : car.redline + 50;
    %             if EngineTorque(j) + .001 < car.gears(i + 1) / car.gears(i) * interp1(car.RPM, car.Torque,j * car.gears(i + 1) / car.gears(i), 'spline')
    %                 ShiftPoints(i) = j;
    %                 break;
    %             end
    %             if j > car.redline
    %                 ShiftPoints(i) = j;
    %                 break;
    %             end
    %         end
    %     end
    %
    %     ShiftPoints(length(ShiftPoints) + 1) = car.redline;
    
    %% Other relevant parameters    
    dRPM            = 0 : 50 : car.redline;
    SmoothTorque    = interp1(car.RPM, car.Torque, dRPM, 'pchip');
    
    %% Shifting Points
    for i = 1 : length(GearReduct)
        speeds(i, :) = ((dRPM / 60) / GearReduct(i)) * car.wheel_circum;
    end
    
    sdiff           = speeds - [zeros(1, length(speeds)); speeds(1 : length(GearReduct) - 1, :)];
    thrust          = (GearReduct' * SmoothTorque) / (car.wheel_dia / 2);
    
    for i = 1 : length(GearReduct) - 1
        
        N           = 1000;
        s1(i, :)    = linspace(speeds(i, 1), speeds(i, end), N);
        
        t1(i, :)    = interp1(speeds(i, :), thrust(i, :), s1(i, :), 'pchip');
        t2(i, :)    = interp1(speeds(i + 1, :), thrust(i + 1, :), s1(i, :), 'pchip');
        
        DY2         = t1(i, :) - t2(i, :);
        sDY2        = sign(DY2);
        dDY2        = diff(sDY2);
        
        idx         = find(dDY2);
        idx(idx == 1) = [];
        
        if length(idx) == 0
            ShiftPoints(i)     = car.redline;
            idx             = length(dDY2);
        else
            spd             = s1(i, idx);
            ShiftPoints(i)     = (spd / car.wheel_circum) * GearReduct(i) * 60;
        end
        
        speed(i) = s1(i, idx);
        trst(i) = t1(i, idx);
    end
    
    % If we are an eCar, there is no shift points
    if (car.motorType == 1)
        shiftSpeed = speed;
        disp(sprintf(['Shift RPMs:\t\t',num2str(round(ShiftPoints))]))
    else
        ShiftPoints = car.RPM(end);
        % Just to keep the data structure happy when running cCar...
        shiftSpeed = 0;
        trst = 0;
    end
    
    
    %% Gearing Arrays
    currentGear                 = 1;
    [maxTorque, maxTorqueIndex] = max(SmoothTorque);
    launchRPM                   = dRPM(maxTorqueIndex);
    launchTransitionSpeed       = ((launchRPM / 60) / (GearReduct(1))) * car.wheel_circum;%10;   % Transition from car.cLutch slip to fully engaged, assume peak torque until this speed
    
    for i = 1:length(car.SpeedRangeMPS)
        speed               = car.SpeedRangeMPS(i);
        
        % Deal with aero and DRS
        if car.DRS_Enabled
            drag            = aeroForce(car.DRS_cD, car.frontArea, speed);
            drag_corner     = aeroForce(car.cD, car.frontArea, speed);
            lift            = aeroForce(car.DRS_cL, car.frontArea, speed);
            lift_corner     = aeroForce(car.cL, car.frontArea, speed);
        else
            drag            = aeroForce(car.cD, car.frontArea, speed);
            drag_corner     = aeroForce(car.cD, car.frontArea, speed);
            lift            = aeroForce(car.DRS_cL, car.frontArea, speed);
            lift_corner     = aeroForce(car.cL, car.frontArea, speed);
        end
        
        dragForce(i)        = drag;
        liftForce(i)        = lift;
        speedPlot(i)        = speed;
        
        DragPower(i)        = drag * speed / 1000;          % In kW = Fdrag * N/lbf * speed * mps/mph / 1000W/kW = (kN*M/s) @ MPH
        DragPower_Corner(i) = drag_corner * speed / 1000;   % In kW = Fdrag * N/lbf * speed * mps/mph / 1000W/kW = (kN*M/s) @ MPH
        
        if speed > launchTransitionSpeed
            rpm             = speed * 60 * (GearReduct(currentGear) / car.wheel_circum);
        else
            rpm             = launchRPM;
        end
        
        rollPowerLoss = ((car.Crr * ((car.mass * 9.81)-(lift)) * car.wheel_dia/2) * (2 * pi()) .* rpm / 60) / 1000;
        if (speed > 0)
            rollForce = rollPowerLoss / speed;
        else
            rollForce = 0;
        end
        
        idealDriveForce_noAero(i) = GearReduct(currentGear) * interp1(car.RPM, car.Torque, rpm, 'spline') / (car.wheel_dia / 2);
        driveForce_noAero(i) = GearReduct(currentGear) * eTEfficiency(interp1(car.RPM, car.Torque, rpm, 'spline')/2,rpm)*2 / (car.wheel_dia / 2) - rollForce;
        % eTEfficiency(interp1(car.RPM, car.Torque, rpm, 'spline')/2,rpm)*2
        
        idealDriveForce(i) = idealDriveForce_noAero(i) - drag;
        driveForce(i)   = driveForce_noAero(i) - drag;
        
        gear(i)         = currentGear;
        
        if driveForce(i) < 0
            driveForce(i) = 0;
        end
        if idealDriveForce(i) < 0
            idealDriveForce(i) = 0;
        end
        
        DriveG(i)       = (driveForce(i) / car.mass) / g;
        AccelPower(i)   = (driveForce(i) * speed) / 1000; % In kW
        idealAccelPower(i) = (idealDriveForce(i) * speed) / 1000;
        
        % Shift stuff
        if(currentGear < length(car.gears))
            if (rpm > ShiftPoints(currentGear))
                if currentGear > 1
                    rpm1                        = speedInGear(currentGear) * 60 * (GearReduct(currentGear) / car.wheel_circum);
                    rpm2                        = speedInGear(currentGear) * 60 * (GearReduct(currentGear - 1) / car.wheel_circum);
                    speedDiff(currentGear)      = ((rpm2 - rpm1) / 60) / (GearReduct(currentGear) / car.wheel_circum);
                    minSpeedInGear(currentGear) = speedInGear(currentGear) - speedDiff(currentGear);
                end
                
                if currentGear <= length(GearReduct)
                    currentGear                 = currentGear + 1;
                end
            end
            speedInGear(currentGear)            = speed;     %For tracking speed in each gear
        end
        
        % This is redundant and should be cleaned, but needed now to get eCar calcs working.
        if (car.motorType == 2)
            speedDiff(1)                        = 0;
            minSpeedInGear(1)                   = 0;
            speedInGear(1)                      = speed;
        end
    end
    
    % Take the lower of the drive or the tractive G
    DriveG2                 = min(DriveG, TractiveG);
    
    Output.TractiveG        = TractiveG;
    Output.DriveG           = DriveG2;
    Output.avgSR            = avgSR;
    Output.Force            = TractiveForce;
    Output.Force_noAero     = driveForce_noAero;
    Output.AccelPower       = AccelPower;
    Output.DragPower        = DragPower;
    Output.DragPower_Corner = DragPower_Corner;
    Output.ShiftPoints      = ShiftPoints;
    Output.speedInGear      = speedInGear;
    Output.minSpeedInGear   = minSpeedInGear;
    Output.speedDiff        = speedDiff;
    Output.gear             = gear;
    Output.itr              = itr;    
    Output.idealAccelPower  = idealAccelPower;
    Output.rollPowerLoss    = rollPowerLoss;
    % These are slutty but will allow thrust plotting for now
    Output.plot.thrust      = thrust;
    Output.plot.speeds      = speeds;
    Output.plot.trst        = trst;
    Output.plot.shiftSpeed  = shiftSpeed;
    
    % Save for diagnostics
    % save('mat\tractiveG')
end

%% CHANGELOG
% 141019 - Daniel Wageman
% *Removed all of the old diag plot stuff. Seems to be working, easy to add
%  again later (just plot everything thats been generated).
%
% 141026 - Daniel Wageman
% *Cleanup and speed verification
% *Fixed shift point indexing (was using g) - could have messed some stuff
%  up since g is already used (and used after that loop)
% *Depricating opperspeed outside of this function (and maybe G funs) - Use
%  SpeedRangeMPS from here on out (and SpeedRangeKPH for plotting)
%
% 141029 - Daniel Wageman
% *Completely re-wrote shift point finder. Worked out in external test file,
%  copied in and working now. *should* be robust for weird inputs/states
%  [thrust intersection @ 0 rpm, NON zero initial torque and rpm,
%  intersecting thrust curves, AND non intersecting thrust curves.
% *General cleanup
% *Improved launch rpm finder - didnt have much effect, but it was wrong
%
% 151011 - Daniel Wageman
% Made changes required for Pacjeka model to work now that there are actual
% model files to test run. Removed speed term for now since that was not
% fit in OptimumT
%
% 161024 - Daniel Wageman
% Modified to work correctly with eCar (single gear, shift speeds, etc)
% Removed silly plotting from this function. Plot elsewhere

%% TODO
