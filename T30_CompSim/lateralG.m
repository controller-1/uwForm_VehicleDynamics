% This function calculates the left and right lateral capacity of the car.
% It is passed a G force convergence tolerance and returns LLateralG,
% RLateralG, LHFrontSA, LHRearSA, RHFrontSA, RHRearSA, LLateralForce, and
% RLateralForce. It loads 'Setup' from CarParams which must be run first.
%
% NOTE: This function needs to be fixed. There is some error with the
% expand_mrandim that does not yield the correct peak SA and Fy when in a
% right hand corner (counting for the signs on the IA and SA as denoted
% below) For now, a simple reversal of SA and Fy while using the left hand
% cornering properties will be used. According to analysis in OptimumT (for R25Bs), the
% tires are nearly symmetrical and this should not cause any error. A
% MPAPeakFinder function was started but still requires work.
%
% expand_mrandim([104,1]    Peak
% expand_mrandim([4,1]      Single point
%
% Writen by Daniel Wageman
% Modified 151011
%
function [Output] = lateralG(varargin)
if nargin == 0
    CompSim()
else
    runPac          = varargin{1};
    tire            = varargin{2};
    car             = varargin{3};
    gTol            = varargin{4};
    maxIterations   = varargin{5};
    
    g = 9.81;
    
    % Conversions for using the broken expand_mrandim
    n2lb            = 0.2248089;
    lb2n            = 1 / n2lb;
    
    % Define IA in terms of camber
    % NOTE! From RCVD, IA is + when the top of the tire leans to the right when viewed from behind
    cRIA            = -1;
    car.cLIA        = 1;
    
    leftFrontIA     = car.cLIA * car.staticCamberF;
    leftRearIA      = car.cLIA * car.staticCamberR;
    rightFrontIA    = cRIA * car.staticCamberF;
    rightRearIA     = cRIA * car.staticCamberR;
    
    % Static tire forces before load transfer and aero
    FL_static       = car.mass * g * (car.cgdistance * car.lateralCGDistance);
    FR_static       = car.mass * g * (car.cgdistance * (1 - car.lateralCGDistance));
    RL_static       = car.mass * g * (1 - car.cgdistance) * car.lateralCGDistance;
    RR_static       = car.mass * g * (1 - car.cgdistance) * (1 - car.lateralCGDistance);
    
    % RCVD defines SA to be + on - steering [left hand turn]. Use abs to condition
    
    % Coord System
    % X: + Rear
    % Y: + Right
    % Z: + Up
    
    %% Left Hand Lateral G's
    % % Preallocate matrices for speed
    % REMOVED - used to pre-allocate, profiler showed this was SLOWER
    
    % Set dummy SA for MRA peak finder. Only the sign matters
    DummySA         = 1;
    
    if runPac
        FResult     = Pacejka(tire, 'py', (-car.mass * g / 4), DummySA, 0, rightFrontIA, car.roadCoefficient, 50, 0);
    else
        FResult     = lb2n * expand_mrandim([104,1], car.NDIM4, [(-car.mass * g / 4) * n2lb, DummySA, rightFrontIA, 0, car.roadCoefficient]); % Call to MRA Non-dimensional Tire Model
    end
    FPeak           = FResult(2);
    startG          = .75 * abs(4 * FPeak / (car.mass * g));
    
    % Set first point to start
    AccelG_Old      = startG;
    
    % Loop through all speeds
    i               = 0;
    keepGoing       = 1;
    while ((keepGoing) && (i < length(car.SpeedRangeCoarseMPS)))
        i               = i + 1;
        speed           = car.SpeedRangeCoarseMPS(i);
        
        lift            = aeroForce(car.cL, car.frontArea, speed);
        drag            = aeroForce(car.cD, car.frontArea, speed);
        FAero(i)        = (-(lift * car.COPdistance) - drag * car.COPHeight / car.wheelbase);
        RAero(i)        = (-(lift * (1 - car.COPdistance)) + drag * car.COPHeight / car.wheelbase);
        
        % Iterate through lateral forces
        n               = 0;
        keepSearching   = 1;
        while ((n < maxIterations) && keepSearching)
            n               = n + 1;
            
            FLWT(i)         = car.TLLTD * (car.mass * g) * car.cgheight * AccelG_Old / car.Ftrack;
            RLWT(i)         = (1 - car.TLLTD) * (car.mass * g) * car.cgheight * AccelG_Old / car.Rtrack;
            
            FLZCurrent(i)   = FL_static - (FLWT(i) / 2) + (FAero(i) / 2);
            FRZCurrent(i)   = FR_static + (FLWT(i) / 2) + (FAero(i) / 2);
            RLZCurrent(i)   = RL_static - (RLWT(i) / 2) + (RAero(i) / 2);
            RRZCurrent(i)   = RR_static + (RLWT(i) / 2) + (RAero(i) / 2);
            
            fTestL          = (FLZCurrent(i) + FRZCurrent(i) + RLZCurrent(i) + RRZCurrent(i)) - (FL_static + FR_static + RL_static + RR_static + FAero(i) + RAero(i));
            
            if (abs(fTestL) > .001)
                warning('car.mass balance off - check equations')
            end
            
            leftFrontIA_roll    = leftFrontIA + car.cLIA * car.rollCamberF * AccelG_Old;
            leftRearIA_roll     = leftRearIA + car.cLIA * car.rollCamberR * AccelG_Old;
            rightFrontIA_roll   = rightFrontIA - cRIA * car.rollCamberF * AccelG_Old;
            rightRearIA_roll    = rightRearIA - cRIA * car.rollCamberR * AccelG_Old;
            
            if runPac
%                 Result      = Pacejka(tire,'py',-FRZCurrent(i),DummySA,0,rightFrontIA_roll,car.roadCoefficient,speed,0);
                Result      = Pacejka(tire,'py',-FRZCurrent(i),DummySA,0,rightFrontIA_roll,car.roadCoefficient,tire.LONGVL,0);
            else
                Result      = [1,lb2n] .* expand_mrandim([104,1],car.NDIM4,[-FRZCurrent(i)*n2lb,DummySA,rightFrontIA_roll,0,car.roadCoefficient]); % Call to MRA Nondimensional Tire Model
            end
            FrontAlphaPeak(i)   = Result(1);
            FYPeak              = Result(2);
            FYFrontRight(i)     = cos(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            FXFrontRight(i)     = sin(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            
            if runPac
%                 FYPeak      = Pacejka(tire, 'fy', -FLZCurrent(i), FrontAlphaPeak(i), 0, leftFrontIA_roll, car.roadCoefficient, speed, 0);
                FYPeak      = Pacejka(tire, 'fy', -FLZCurrent(i), FrontAlphaPeak(i), 0, leftFrontIA_roll, car.roadCoefficient, tire.LONGVL, 0);
            else
                FYPeak      = lb2n*expand_mrandim([4,1], car.NDIM4, [-FLZCurrent(i) * n2lb, FrontAlphaPeak(i), leftFrontIA_roll, 0, car.roadCoefficient]);
            end
            FYFrontLeft(i)  = cos(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            FXFrontLeft(i)  = sin(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            
            FFront          = FYFrontRight(i) + FYFrontLeft(i);
            
            if runPac
%                 Result      = Pacejka(tire, 'py', -RRZCurrent(i), DummySA, 0, rightRearIA_roll, car.roadCoefficient, speed, 0);
                Result      = Pacejka(tire, 'py', -RRZCurrent(i), DummySA, 0, rightRearIA_roll, car.roadCoefficient, tire.LONGVL, 0);
            else
                Result      = [1, lb2n] .* expand_mrandim([104,1], car.NDIM4, [-RRZCurrent(i) * n2lb, DummySA, rightRearIA_roll, 0, car.roadCoefficient]); % Call to MRA Nondimensional Tire Model
            end
            RearAlphaPeak(i)    = Result(1);
            FYPeak              = Result(2);
            FYRearRight(i)      = cos(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            FXRearRight(i)      = sin(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            
            if runPac
%                 FYPeak      = Pacejka(tire,'fy',-RLZCurrent(i),RearAlphaPeak(i),0,leftRearIA_roll,car.roadCoefficient,speed,0);
                FYPeak      = Pacejka(tire,'fy',-RLZCurrent(i),RearAlphaPeak(i),0,leftRearIA_roll,car.roadCoefficient,tire.LONGVL,0);
            else
                FYPeak      = lb2n*expand_mrandim([4,1],car.NDIM4,[-RLZCurrent(i)*n2lb,RearAlphaPeak(i),leftRearIA_roll,0,car.roadCoefficient]);
            end
            FYRearLeft(i)   = cos(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            FXRearLeft(i)   = sin(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            
            FRear           = FYRearRight(i) + FYRearLeft(i);
            
            AccelG_New      = (abs(FFront) + abs(FRear)) / (car.mass * g);
            difference      = abs(AccelG_Old - AccelG_New);
            AccelG_Old      = AccelG_New;
            
            if (difference < gTol)
                itr(1,i)        = n;
                LLateralG(i)    = AccelG_New;
                LHFrontSA(i)    = FrontAlphaPeak(i);
                LHRearSA(i)     = RearAlphaPeak(i);
                LMoment(i)      = (abs(FFront) * car.cgdistance * car.wheelbase) - (abs(FRear) * (1 - car.cgdistance) * car.wheelbase);
                keepSearching   = 0;
            end
            
            if n == maxIterations
                warning('Max LLateralG Iterations Reached! Solution may be incorrect. Try changing maxIttr or check setup.')
                keepGoing = 0;
            end
            
        end
        
        % Collect all X,Y,Z forces for left hand turns
        LLateralForce(:,i,1)    = [FXFrontLeft(i);FYFrontLeft(i);FLZCurrent(i)];      % FL
        LLateralForce(:,i,2)    = [FXFrontRight(i);FYFrontRight(i);FRZCurrent(i)];     % FR
        LLateralForce(:,i,3)    = [FXRearLeft(i);FYRearLeft(i);RLZCurrent(i)];       % RL
        LLateralForce(:,i,4)    = [FXRearRight(i);FYRearRight(i);RRZCurrent(i)];      % RR
        LLateralForce(1,i,5)    = [LMoment(i)];
    end
    
    % Adjust for driver performance
    LLateralG       = LLateralG .* car.driverCorneringCoP;
    
    %% Right Hand Lateral G's
    % REMOVED - used to pre-allocate, profiler showed this was SLOWER
    
    % Set dummy SA for MRA peak finder. Only the sign matters
    % When done according to MRA coord systems, this should be - for RH
    % cornering. Set to positive for now
    DummySA         = -1;
    
    if runPac
        FResult     = Pacejka(tire, 'py', (-car.mass * g / 4), -DummySA, 0, rightFrontIA, car.roadCoefficient, 50, 0);
    else
        FResult     = lb2n * expand_mrandim([104,1], car.NDIM4, [(-car.mass * g / 4) * n2lb, -DummySA, rightFrontIA, 0, 0, car.roadCoefficient]); % Call to MRA Non-dimensional Tire Model
    end
    FPeak           = FResult(2);
    startG          = .75 * abs(4 * FPeak / (car.mass * g));
    
    AccelG_Old      = startG;
    
    % Loop through all speeds
    i               = 0;
    keepGoing       = 1;
    while ((keepGoing) && (i < length(car.SpeedRangeCoarseMPS)))
        i               = i + 1;
        speed           = car.SpeedRangeCoarseMPS(i);
        
        lift            = aeroForce(car.cL, car.frontArea, speed);
        drag            = aeroForce(car.cD, car.frontArea, speed);
        FAero(i)        = (-(lift * car.COPdistance) - drag * car.COPHeight / car.wheelbase);
        RAero(i)        = (-(lift * (1 - car.COPdistance)) + drag * car.COPHeight / car.wheelbase);
        
        % Iterate through lateral forces
        n               = 0;
        keepSearching   = 1;
        while ((n < maxIterations) && keepSearching)
            n               = n + 1;
            
            FLWT(i)         = car.TLLTD * (car.mass * g) * car.cgheight * AccelG_Old / car.Ftrack;
            RLWT(i)         = (1 - car.TLLTD) * (car.mass * g) * car.cgheight * AccelG_Old / car.Rtrack;
            
            FLZCurrent(i)   = FL_static + (FLWT(i) / 2) + (FAero(i) / 2);
            FRZCurrent(i)   = FR_static - (FLWT(i) / 2) + (FAero(i) / 2);
            RLZCurrent(i)   = RL_static + (RLWT(i) / 2) + (RAero(i) / 2);
            RRZCurrent(i)   = RR_static - (RLWT(i) / 2) + (RAero(i) / 2);
            
            fTestR          = (FLZCurrent(i) + FRZCurrent(i) + RLZCurrent(i) + RRZCurrent(i)) - (FL_static + FR_static + RL_static + RR_static + FAero(i) + RAero(i));
            
            if (abs(fTestR) > .001)
                warning('car.mass balance off - check equations')
            end
            
            % Previous code and comment here was due to a misunderstanding of how the MRA code was called and what it returned.
            % The tires are tested and modelled ONLY on side of the car. The signs are simply flipped on the result assuming equal and opposite SA and IA.
            
            leftFrontIA_roll    = leftFrontIA + car.cLIA * car.rollCamberF * AccelG_Old;
            leftRearIA_roll     = leftRearIA + car.cLIA * car.rollCamberR * AccelG_Old;
            rightFrontIA_roll   = rightFrontIA - cRIA * car.rollCamberF * AccelG_Old;
            rightRearIA_roll    = rightRearIA - cRIA * car.rollCamberR * AccelG_Old;
            
            if runPac
%                 Result      = Pacejka(tire,'py',-FLZCurrent(i),-DummySA,0,rightFrontIA_roll,car.roadCoefficient,speed,0);
                Result      = Pacejka(tire,'py',-FLZCurrent(i),-DummySA,0,rightFrontIA_roll,car.roadCoefficient,tire.LONGVL,0);
            else
                Result      = [1,lb2n] .* expand_mrandim([104,1],car.NDIM4,[-FLZCurrent(i)*n2lb,-DummySA,rightFrontIA_roll,0,car.roadCoefficient]); % Call to MRA Nondimensional Tire Model
            end
            FrontAlphaPeak(i)   = Result(1);
            FYPeak              = Result(2);
            FYFrontLeft(i)      = -cos(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            FXFrontLeft(i)      = sin(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            
            if runPac
%                 FYPeak      = Pacejka(tire, 'fy', -FRZCurrent(i), FrontAlphaPeak(i), 0, leftFrontIA_roll, car.roadCoefficient, speed, 0);
                FYPeak      = Pacejka(tire, 'fy', -FRZCurrent(i), FrontAlphaPeak(i), 0, leftFrontIA_roll, car.roadCoefficient, tire.LONGVL, 0);
            else
                FYPeak      = lb2n * expand_mrandim([4,1], car.NDIM4, [-FRZCurrent(i) * n2lb, FrontAlphaPeak(i), leftFrontIA_roll, 0, car.roadCoefficient]);
            end
            FYFrontRight(i) = -cos(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            FXFrontRight(i) = sin(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            
            FFront          = FYFrontRight(i) + FYFrontLeft(i);
            
            if runPac
%                 Result      = Pacejka(tire, 'py', -RLZCurrent(i), -DummySA, 0, rightRearIA_roll, car.roadCoefficient, speed, 0);
                Result      = Pacejka(tire, 'py', -RLZCurrent(i), -DummySA, 0, rightRearIA_roll, car.roadCoefficient, tire.LONGVL, 0);
            else
                Result      = [1, lb2n] .* expand_mrandim([104,1], car.NDIM4, [-RLZCurrent(i) * n2lb, -DummySA, rightRearIA_roll, 0, car.roadCoefficient]); % Call to MRA Nondimensional Tire Model
            end
            RearAlphaPeak(i)    = Result(1);
            FYPeak              = Result(2);
            FYRearLeft(i)       = -cos(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            FXRearLeft(i)       = sin(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            
            if runPac
%                 FYPeak      = Pacejka(tire, 'fy', -RRZCurrent(i), RearAlphaPeak(i), 0, leftRearIA_roll, car.roadCoefficient, speed, 0);
                FYPeak      = Pacejka(tire, 'fy', -RRZCurrent(i), RearAlphaPeak(i), 0, leftRearIA_roll, car.roadCoefficient, tire.LONGVL, 0);
            else
                FYPeak      = lb2n * expand_mrandim([4,1], car.NDIM4, [-RRZCurrent(i) * n2lb, RearAlphaPeak(i), leftRearIA_roll, 0, car.roadCoefficient]);
            end
            FYRearRight(i)  = -cos(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            FXRearRight(i)  = sin(FrontAlphaPeak(i) * pi / 180) * FYPeak;
            
            FRear           = FYRearRight(i) + FYRearLeft(i);
            
            AccelG_New      =    (abs(FFront) + abs(FRear)) / (car.mass * g);
            difference      =    abs(AccelG_Old - AccelG_New);
            AccelG_Old      =    AccelG_New;
            
            if (difference < gTol)
                itr(2,i)        = n;
                RLateralG(i)    = AccelG_New;
                RHFrontSA(i)    = -FrontAlphaPeak(i);
                RHRearSA(i)     = -RearAlphaPeak(i);
                RMoment(i)      = (abs(FFront) * car.cgdistance * car.wheelbase) - (abs(FRear) * (1 - car.cgdistance) * car.wheelbase);
                keepSearching   = 0;
            end
            
            if n==maxIterations
                warning('Max RLateralG Iterations Reached! Solution may be incorrect. Try changing maxIttr or check setup.')
                keepGoing = 0;
            end
        end
        
        % Collect all X,Y,Z forces for left hand turns
        RLateralForce(:,i,1)    = [FXFrontLeft(i);FYFrontLeft(i);FLZCurrent(i)];      % FL
        RLateralForce(:,i,2)    = [FXFrontRight(i);FYFrontRight(i);FRZCurrent(i)];     % FR
        RLateralForce(:,i,3)    = [FXRearLeft(i);FYRearLeft(i);RLZCurrent(i)];       % RL
        RLateralForce(:,i,4)    = [FXRearRight(i);FYRearRight(i);RRZCurrent(i)];      % RR
        RLateralForce(1,i,5)    = [RMoment(i)];
    end
    
    % Adjust for driver performance
    RLateralG       = RLateralG .* car.driverCorneringCoP;
    
    Output.LG       = LLateralG;
    Output.RG       = RLateralG;
    Output.LForce   = LLateralForce;
    Output.RForce   = RLateralForce;
    Output.itr      = itr;
    
    % Save for diagnostics
    % save('mat\lateralG')
    % toc
end

%% CHANGELOG
% 141019 - Daniel Wageman
% Removed all of the old diag plot stuff. Seems to be working, easy to add
% again later (just plot everything thats been generated).
%
% 141026 - Daniel Wageman
% Cleanup and speed check
% Depricating opperspeed outside of this function (and maybe G funs) - Use
% SpeedRangeMPS from here on out (and SpeedRangeKPH for plotting)
%
% 151011 - Daniel Wageman
% Made changes required for Pacjeka model to work now that there are actual
% model files to test run. Removed speed term for now since that was not
% fit in OptimumT
