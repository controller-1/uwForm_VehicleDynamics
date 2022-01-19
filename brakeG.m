% This function calculates the braking capacity of the car. It is passed a
% G force convergence tolerance and returns BrakeG, regenPower, BrakeForce,
% ReverseBrakeG, ReverseBrakeForce, AeroF, and ReverseAeroF. All in MPS
%
% Writen by Daniel Wageman
% Updated 161024
%
function [Output]=brakeG(varargin)
if nargin == 0
    CompSim()
else
    runPac          = varargin{1};
    tire            = varargin{2};
    car             = varargin{3};
    gTol            = varargin{4};
    maxIterations   = varargin{5};
    
    g               = 9.81;
    
    maxRegenkW      = car.contRegenkW;
    
    % Define IA in terms of camber
    % NOTE! From RCVD, IA is + when the top of the tire leans to the right when viewed from behind
    cRIA            = -1;
    cLIA            = 1;
    
    leftFrontIA     = cLIA * abs(car.staticCamberF);
    rightFrontIA    = cRIA * abs(car.staticCamberF);
    leftRearIA      = cLIA * abs(car.staticCamberR);
    rightRearIA     = cRIA * abs(car.staticCamberR);
    
    % Static tire forces before load transfer and aero
    FL_static       = car.mass * g * (car.cgdistance * car.lateralCGDistance);
    FR_static       = car.mass * g * (car.cgdistance * (1 - car.lateralCGDistance));
    RL_static       = car.mass * g * (1 - car.cgdistance) * car.lateralCGDistance;
    RR_static       = car.mass * g * (1 - car.cgdistance) * (1 - car.lateralCGDistance);
    
    % Preallocate matrices for speed
    % REMOVED - actually slows it down in this case
    BrakeG          = zeros(1, length(car.SpeedRangeCoarseMPS));
    
    % Set dummy slip ratio for peak finder. Only sign matters
    DummySR         = -1;
    
    % Bootstrap things to get started
    if runPac
        FResult     = -Pacejka(tire, 'px', (-car.mass * g / 4), 0, DummySR, rightFrontIA, car.roadCoefficient, tire.LONGVL);
    else
        FResult     = -expand_mrandim([106,2], car.NDIM6, [-car.mass * g / 4, 0, rightFrontIA, DummySR, car.roadCoefficient]); % Call to MRA Nondimensional Tire Model
    end
    
    FPeak           = FResult(2);
    
    % Start the itteration off at ~90% level car peak
    startG          = .9 * abs(4 * FPeak / (car.mass * g));
    
    % Set first point to start
    AccelG_Old      = startG;
    
    i               = 0;
    keepGoing       = 1;
    % Loop through all speeds
    while ((i < length(car.SpeedRangeCoarseMPS)) && (keepGoing))
        i               = i + 1;
        speed           = car.SpeedRangeCoarseMPS(i);
        
        aeroLift(i)     = aeroForce(car.cL, car.frontArea, speed);
        aeroDrag(i)     = aeroForce(car.cD, car.frontArea, speed);
        frontFZAero(i)  = (-(aeroLift(i) * car.COPdistance) - aeroDrag(i) * car.COPHeight / car.wheelbase);
        rearFZAero(i)   = (-(aeroLift(i) * (1 - car.COPdistance)) + aeroDrag(i) * car.COPHeight / car.wheelbase);
        
        n               = 0;
        keepSearching   = 1;
        while ((n < maxIterations) && keepSearching)
            n               = n + 1;
            
            LongWT          = car.mass * car.cgheight * (AccelG_Old * g) / car.wheelbase;
            
            FLFZCurrent     = FL_static + (LongWT / 2) + (frontFZAero(i) / 2);
            FRFZCurrent     = FR_static + (LongWT / 2) + (frontFZAero(i) / 2);
            RLFZCurrent     = RL_static - (LongWT / 2) + (rearFZAero(i) / 2);
            RRFZCurrent     = RR_static - (LongWT / 2) + (rearFZAero(i) / 2);
            
            % Make sure the car has not lost any mystery mass
            fTest           = (FLFZCurrent + FRFZCurrent + RLFZCurrent + RRFZCurrent) - (FL_static + FR_static + RL_static + RR_static + frontFZAero(i) + rearFZAero(i));
            
            if (abs(fTest) > .001)
                warning('car.mass balance off - check equations')
            end
            
            % New IAs due to camber gain
            leftFrontIA_pitch   = leftFrontIA - cLIA * car.pitchCamberF * AccelG_Old;
            rightFrontIA_pitch  = rightFrontIA - cRIA * car.pitchCamberF * AccelG_Old;
            rightRearIA_pitch   = rightRearIA + cRIA * car.pitchCamberR * AccelG_Old;
            leftRearIA_pitch    = leftRearIA + cLIA * car.pitchCamberR * AccelG_Old;
            
            if runPac
%                 FLResult    = -Pacejka(tire, 'px', -FLFZCurrent, 0, DummySR, leftFrontIA_pitch, car.roadCoefficient, speed);
%                 FRResult    = -Pacejka(tire, 'px', -FRFZCurrent, 0, DummySR, rightFrontIA_pitch, car.roadCoefficient, speed);
                FLResult    = -Pacejka(tire, 'px', -FLFZCurrent, 0, DummySR, leftFrontIA_pitch, car.roadCoefficient, tire.LONGVL);
                FRResult    = -Pacejka(tire, 'px', -FRFZCurrent, 0, DummySR, rightFrontIA_pitch, car.roadCoefficient, tire.LONGVL);
            else
                FLResult    = -expand_mrandim ([106,2], car.NDIM6, [-FLFZCurrent, 0, leftFrontIA_pitch, DummySR, car.roadCoefficient]);
                FRResult    = -expand_mrandim ([106,2], car.NDIM6, [-FRFZCurrent, 0, rightFrontIA_pitch, DummySR, car.roadCoefficient]);
            end
            FLx             = FLResult(2);
            FRx             = FRResult(2);
            
            FxFront         = FRx + FLx;
            
            if runPac
%                 RLResult    = -Pacejka(tire, 'px', -RLFZCurrent, 0, DummySR, leftRearIA_pitch, car.roadCoefficient, speed);
%                 RRResult    = -Pacejka(tire, 'px', -RRFZCurrent, 0, DummySR, rightRearIA_pitch, car.roadCoefficient, speed);
                RLResult    = -Pacejka(tire, 'px', -RLFZCurrent, 0, DummySR, leftRearIA_pitch, car.roadCoefficient, tire.LONGVL);
                RRResult    = -Pacejka(tire, 'px', -RRFZCurrent, 0, DummySR, rightRearIA_pitch, car.roadCoefficient, tire.LONGVL);
            else
                RLResult    = -expand_mrandim ([106,2], car.NDIM6, [-RLFZCurrent, 0, leftRearIA_pitch, DummySR, car.roadCoefficient]);
                RRResult    = -expand_mrandim ([106,2], car.NDIM6, [-RRFZCurrent, 0, rightRearIA_pitch, DummySR, car.roadCoefficient]);
            end
            RLx             = RLResult(2);
            RRx             = RRResult(2);
            
            FxRear          = RRx + RLx;
            
            FxTotal         = FxFront + FxRear;
            
            % Take fixed Brake Bias into account
            if (car.fixedBrakeBias)
                if (FxFront > (FxTotal * car.brakeBias))
                    FxFront = FxTotal * car.brakeBias;
                end
                if (FxRear > (FxTotal * (1 - car.brakeBias)))
                    FxRear  = FxTotal * (1 - car.brakeBias);
                end
                FxTotal     = FxFront + FxRear;
            end
            
            AccelG_New      = FxTotal / (car.mass * g);
            difference      = abs(AccelG_Old - AccelG_New);
            AccelG_Old      = AccelG_New;
            
            if (difference < gTol)
                keepSearching = 0;
            end
            
            if n == maxIterations
                warning([num2str(n),' Iterations at ',num2str(speed),' kph. Max brakeG Iterations Exceed! Solution may be incorrect. Try changing maxIttr or check setup.'])
                keepGoing = 0;
            end
            
        end
        
        % Keep Results
        itr(i)              = n;
        BrakeG(i)           = AccelG_New + (aeroDrag(i) / (car.mass * g));
        Bias(i)             = FxFront ./ (FxTotal);
        regenPower(i)       = FxRear * speed / 1000; %min(FxRear * speed / 1000, maxRegenkW);       % Power in kW
        Force.Front(i)      = FxFront;
        Force.Rear(i)       = FxRear;
                
        BrakeForce(:,i,1)   = [FLx; 0; FLFZCurrent];      % FL
        BrakeForce(:,i,2)   = [FRx; 0; FRFZCurrent];      % FR
        BrakeForce(:,i,3)   = [RLx; 0; RLFZCurrent];      % RL
        BrakeForce(:,i,4)   = [RRx; 0; RRFZCurrent];      % RR
    end
    
    % Adjust for driver performance
    BrakeG              = BrakeG .* car.driverBrakingCoP;
    
    % Save aero forces
    AeroF               = [frontFZAero; rearFZAero];
    
    %% Reverse Force Calc
    
    % Set dummy slip ratio for peak finder. Only sign matters
    % The MRA code yields a higher Fx when this value is set to +SR even though
    % the SR is - when running backward. Since this is used to calculate load
    % cases, we will assume the higher value.
    DummySR         = .1;
    
    if runPac
        FResult     = -Pacejka(tire, 'px', (-car.mass * g / 4), 0, DummySR, rightRearIA, car.roadCoefficient, 50);
    else
        FResult     = -expand_mrandim([106,2], car.NDIM6, [-car.mass * g / 4, 0, rightRearIA, DummySR, car.roadCoefficient]); % Call to MRA Non-dimensional Tire Model
    end
    FPeak           = FResult(2);
    startG          = .9 * abs(4 * FPeak / (car.mass * g));
    
    % Set first point to start
    AccelG_Old      = startG;
    
    % Loop through all speeds
    % for i=1:length(car.SpeedRangeCoarseMPS)
    %     speed=car.SpeedRangeCoarseMPS(i);
    i               = 0;
    keepGoing       = 1;
    while ((keepGoing) && (i < length(car.SpeedRangeCoarseMPS)))
        i               = i + 1;
        speed           = car.SpeedRangeCoarseMPS(i);
        
        aeroLift(i)     = aeroForce(car.rev_cL, car.frontArea, speed);
        aeroDrag(i)     = aeroForce(car.rev_cD, car.frontArea, speed);
        frontFZAero(i)  = (-(aeroLift(i) * car.COPdistance) + aeroDrag(i) * car.COPHeight / car.wheelbase);
        rearFZAero(i)   = (-(aeroLift(i) * (1 - car.COPdistance)) - aeroDrag(i) * car.COPHeight / car.wheelbase);
        
        %Iterate through lateral forces
        n               = 0;
        keepSearching   = 1;
        while ((n < maxIterations) && keepSearching)
            n               = n + 1;
            
            LongWT          = car.mass * car.cgheight * (AccelG_Old * g) / car.wheelbase;
            
            FLFZCurrent     = FL_static - (LongWT / 2) + (frontFZAero(i) / 2);
            FRFZCurrent     = FR_static - (LongWT / 2) + (frontFZAero(i) / 2);
            RLFZCurrent     = RL_static + (LongWT / 2) + (rearFZAero(i) / 2);
            RRFZCurrent     = RR_static + (LongWT / 2) + (rearFZAero(i) / 2);
            
            fTest           = (FLFZCurrent + FRFZCurrent + RLFZCurrent + RRFZCurrent) - (FL_static + FR_static + RL_static + RR_static + frontFZAero(i) + rearFZAero(i));
            
            if (abs(fTest) > .001)
                warning('car.mass balance off - check equations')
            end
            
            rightFrontIA_pitch  = rightFrontIA + cRIA * car.pitchCamberF * AccelG_Old;
            leftFrontIA_pitch   = leftFrontIA + cLIA * car.pitchCamberF * AccelG_Old;
            rightRearIA_pitch   = rightRearIA - cRIA * car.pitchCamberR * AccelG_Old;
            leftRearIA_pitch    = leftRearIA - cLIA * car.pitchCamberR * AccelG_Old;
            
            if runPac
%                 FLResult    = -Pacejka(tire, 'px', -FLFZCurrent, 0, DummySR, leftFrontIA_pitch, car.roadCoefficient, speed);
%                 FRResult    = -Pacejka(tire, 'px', -FRFZCurrent, 0, DummySR, rightFrontIA_pitch, car.roadCoefficient, speed);
                FLResult    = -Pacejka(tire, 'px', -FLFZCurrent, 0, DummySR, leftFrontIA_pitch, car.roadCoefficient, tire.LONGVL);
                FRResult    = -Pacejka(tire, 'px', -FRFZCurrent, 0, DummySR, rightFrontIA_pitch, car.roadCoefficient, tire.LONGVL);
            else
                FLResult    = -expand_mrandim ([106,2], car.NDIM6, [-FLFZCurrent, 0, leftFrontIA_pitch, DummySR, car.roadCoefficient]);
                FRResult    = -expand_mrandim ([106,2], car.NDIM6, [-FRFZCurrent, 0, rightFrontIA_pitch, DummySR, car.roadCoefficient]);
            end
            FLx             = FLResult(2);
            FRx             = FRResult(2);
            
            FxFront         = FRx + FLx;
            
            if runPac
%                 RLResult    = -Pacejka(tire, 'px', -RLFZCurrent, 0, DummySR, leftRearIA_pitch, car.roadCoefficient, speed);
%                 RRResult    = -Pacejka(tire, 'px', -RRFZCurrent, 0, DummySR, rightRearIA_pitch, car.roadCoefficient, speed);
                RLResult    = -Pacejka(tire, 'px', -RLFZCurrent, 0, DummySR, leftRearIA_pitch, car.roadCoefficient, tire.LONGVL);
                RRResult    = -Pacejka(tire, 'px', -RRFZCurrent, 0, DummySR, rightRearIA_pitch, car.roadCoefficient, tire.LONGVL);
            else
                RLResult    = -expand_mrandim ([106,2], car.NDIM6, [-RLFZCurrent, 0, leftRearIA_pitch, DummySR, car.roadCoefficient]);
                RRResult    = -expand_mrandim ([106,2], car.NDIM6, [-RRFZCurrent, 0, rightRearIA_pitch, DummySR, car.roadCoefficient]);
            end
            RLx             = RLResult(2);
            RRx             = RRResult(2);
            
            FxRear          = RRx + RLx;
            
            FxTotal         = FxFront + FxRear;
            
            % Take fixed Brake Bias into account
            if (car.fixedBrakeBias)
                if (FxFront > (FxTotal * car.brakeBias))
                    FxFront = FxTotal * car.brakeBias;
                end
                if (FxRear > (FxTotal * (1 - car.brakeBias)))
                    FxRear  = FxTotal * (1 - car.brakeBias);
                end
                FxTotal     = FxFront + FxRear;
            end

            AccelG_New      = FxTotal / (car.mass * g);
            difference      = abs(AccelG_Old - AccelG_New);
            AccelG_Old      = AccelG_New;
            
            if (difference < gTol)
                keepSearching = 0;
            end
            
            if n == maxIterations
                warning('Max Reverse brakeG Iterations Reached! Solution may be incorrect. Try changing maxIttr or check setup.')
                keepGoing = 0;
            end
            
        end
        
        ReverseBrakeG(i)            = AccelG_New + (aeroDrag(i) / (car.mass * g));
        ReverseBrakeForce(:,i,1)    = [FLx; 0; FLFZCurrent];      % FL
        ReverseBrakeForce(:,i,2)    = [FRx; 0; FRFZCurrent];      % FR
        ReverseBrakeForce(:,i,3)    = [RLx; 0; RLFZCurrent];      % RL
        ReverseBrakeForce(:,i,4)    = [RRx; 0; RRFZCurrent];      % RR
    end
    
    % Save reverse aero FZ forces
    ReverseAeroF        = [frontFZAero; rearFZAero];
    
    % Setup Output Structure
    Output.G            = BrakeG;
    Output.regenPower   = regenPower;
    Output.BForce       = BrakeForce;
    Output.ReverseG     = ReverseBrakeG;
    Output.ReverseForce = ReverseBrakeForce;
    Output.AeroF        = AeroF;
    Output.ReverseAeroF = ReverseAeroF;
    Output.drag         = aeroDrag;
    Output.itr          = itr;
    Output.Bias         = Bias;
    Output.Force        = Force;

    % Save for diagnostics
    % save('mat\brakeG')
    
end

%% CHANGELOG
% 141015 - Daniel Wageman
% Added comments and cleaned up
% Fixed LongWT in reverse braking condition, should LESSEN F Wheel Load
% Save more outputs to structure for use elsewhere (brake temp)
% Changed FLFZCurrent name structure to FLFFZCurrent
% Removed diagPlots - simple and can add later
% Optimized loop operations as much as possible
%
% 141019 - Daniel Wageman
% Removed all of the old diag plot stuff. Seems to be working, easy to add
% again later (just plot everything thats been generated).
%
% 141026 - Daniel Wageman
% Cleanup and speed verification
% Depricating opperspeed outside of this function (and maybe G funs) - Use
% SpeedRangeMPS from here on out (and SpeedRangeKPH for plotting)
%
% 151011 - Daniel Wageman
% Made changes required for Pacjeka model to work now that there are actual
% model files to test run. Removed speed term for now since that was not
% fit in OptimumT
%
% 161024 - Daniel Wageman
% Regen Power updated slightly and comments

