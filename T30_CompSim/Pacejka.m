% Pacejka.m()
%
% This function calculates resultant tire forces using Pacejka coefficients
% imported from an external file. Type is a string of 'Fy', 'Fx', or 'Fc' 
% where Fz is the normal tire force (- is downward), SA is slip angle IN 
% DEGREES, SR is slip ratio in %, IA is inclination angle, and mu is the 
% friction coefficient knockdown.
%
% This function is intended to closely imitate the propietary MRANDIM
% function provided with the round 3 FSAE TTC data. Tire data must be
% provided in the form of a Pacejka 2006 or greater .tir file.
%
% As of fall 2015, Pressure Effects are noted in the equations but NOT
% included in the computation. Once pressure based models are fit, the
% effects can be turned on.
%
% All equation references are relative to "Tire and Vehicle Dynamics" by
% Hans Pacejka, Third Edition
%
% Written by Daniel Wageman
% BSME, MSME University of Washington
% Updated 151011
%
% ******* TODO *******
% Add pressure term
% Add velocity term
%
% ******* Changelog *******
% 151011 Fixed significant errors in FY calculation regarding dfz and Fzp.
%        Included aditional scaling factors as per book.
%        Update peak finders for more natural use

function result = Pacejka(t,type,Fz,SA,SR,IA,mu,V,tF)
% Import tire data
% The following commands must have been run first by calling function:
% tire = ImportTireData('TIREFILE.tir');

% Settings
% Print range warnings to screen
warnOn      = 0;
% Check for range of requested inputs vs test range. Warn user if exceded
checkRng    = 1;
% Set limit on peak finding and reverse lookup itteration counts
maxItr      = 50;
% For safety and sanity, limit the output SA and SR peak finders to some
% multiple of the max tested value
peakLimit   = 2;

% Rename variables for convenience when applying Pacejka's formulas direcly from the book.
alpha       = (SA / 180) * pi();

gamma       = (IA/180) * pi();
% If mu is non-zero, use given mu, otherwise use mu defined in file
if mu == 0
    LMuY        = t.LMUY;       % Peak friction coef
    LMuX        = t.LMUX;       % Peak friction coef
else
    LMuY        = mu;           % Peak friction coef
    LMuX        = mu;           % Peak friction coef
end
% LMuV        = 0;              % Speed with slip decaying function, 0 if not used
LMuV        = t.LMUV;           % Speed with slip decaying function, 0 if not used
AMu         = 10;
K           = SR;
t.PDX3      = 0;                % 0 for now, not part of earlier models

% Global variables
Eps         = .001;             % To prevent divergence for small denom

if checkRng
    % Check inputs for validity
    % Long slip range
    if ((max(SR) > t.KPUMAX) || (min(SR) < t.KPUMIN))
        if(warnOn);warning('Slip ratio out of test data range, verify validity');end
    end
    
    % Slip angle range
    if ((max(alpha) > t.ALPMAX) || (min(alpha) < t.ALPMIN))
        if(warnOn);warning('Slip angle out of test data range, verify validity');end
    end
    
    % Inclination angle range
    if (gamma > t.CAMMAX || gamma < t.CAMMIN)
        if(warnOn);warning('Camber out of test data range, verify validity');end
    end
    
    % Vertical force range
    if (abs(Fz) > t.FZMAX || abs(Fz) < t.FZMIN)
        if(warnOn);warning('Fz out of test data range, verify validity');end
    end
end

switch lower(type)
    case {'fx'}
        % Fx solution
        % (4.E1)
        Fzp         = t.LFZ0 * Fz;
        
        % (4.E2a)
        dfz         = (Fzp - t.FNOMIN) ./ t.FNOMIN;
        
        % (4.E2b) Normalized pressure
        % dpi         = (Pi - Pio) / Pio;
       
        % (4.E7)
        LStarMuX    = LMuX ./ (1 + (LMuV .* V) / t.LONGVL);
        
        % (4.E8)
        LPMuX       = AMu * LStarMuX / (1 + (AMu - 1) * LStarMuX);
        
        % (4.E17)
        Shx         = (t.PHX1 + t.PHX2 .* dfz) * t.LHX;
        
        % (4.E10)
        Kx          = (K + Shx);
        
        % (4.E11)
        Cx          = t.PCX1 * t.LCX;
        
        % (4.E13) No pressure terms
        muX         = (t.PDX1 + t.PDX2 .* dfz) * (1 - t.PDX3 * gamma^2) * LStarMuX;
        % (4.E13) With pressure terms
        % muX         = (t.PDX1 + t.PDX2 .* dfz) * (1 + t.PPX3 * dpi + t.PPX4 * dpi ^ 2) * (1 - t.PDX3 * gamma^2) * LStarMuX;
        
        % (4.E12)
        Dx          = (muX .* Fz);
        
        % (4.E14)
        Ex          = ((t.PEX1 + t.PEX2 .* dfz + t.PEX3 .* dfz.^2) .* (1 - t.PEX4 * sign(Kx))) * t.LEX;
        
        % (4.E15) No pressure terms
        Kxk         = (Fz .* (t.PKX1 + t.PKX2 .* dfz) .* exp(t.PKX3 .* dfz));
        % (4.E15) With pressure terms
        % Kxk         = (Fz .* (t.PKX1 + t.PKX2 .* dfz) .* exp(t.PKX3 .* dfz)) * (1 + t.PPX1 * dpi + t.PPX2 * dpi ^ 2);

        % (4.E16)
        Bx          = (Kxk ./ (Cx * Dx + Eps));
        
        % (4.E18)
        Svx         = Fz .* (t.PVX1 + t.PVX2 .* dfz) * t.LVX * LPMuX;
        
        % (4.E9)
        Fx          = (Dx .* sin(Cx .* atan((Bx .* Kx - Ex .* (Bx .* Kx - atan(Bx .* Kx))))) + Svx);
        
        result      = Fx;
        
    case {'fy'}
        % Fy solution
        % (4.E1)
        Fzp         = t.LFZ0 * Fz;
        
        % (4.E2a)
        dfz         = (Fzp - t.FNOMIN) ./ t.FNOMIN;
        
        % (4.E2b) Normalized pressure
        % dpi         = (Pi - Pio) / Pio;
        
        % (4.E3)
        alphaStar   = (tan(alpha) .* sign(V))';
        
        % (4.E4)
        gammaStar   = sin(gamma);
        
        % (4.E7)
        LStarMuY    = LMuY ./ (1 + (LMuV .* V) / t.LONGVL);
        
        % (4.E8)
        LPMuY       = AMu * LStarMuY / (1 + (AMu - 1) * LStarMuY);
        
        % (4.E23) No pressure terms
        muY         = (t.PDY1 + t.PDY2 .* dfz) .* (1 - t.PDY3 .* gammaStar .^ 2) .* LStarMuY;
        % (4.E23) With pressure terms
        % muY         = (t.PDY1 + t.PDY2 .* dfz) .* (1 + t.PDY3 * dpi + t.PDY4 * dpi ^ 2) .* (1 - t.PDY3 .* gammaStar .^ 2) .* LStarMuY;
        
        % (4.E22)
        Dy          = muY .* Fz;
        
        % (4.E21)
        Cy          = t.PCY1 * t.LCY;
        
        % (4.E28)
        Svygamma    = Fz .* (t.PVY3 + t.PVY4 .* dfz) .* gammaStar * t.LKYC .* LPMuY;
        
        % (4.E29)
        Svy         = Fz .* (t.PVY1 + t.PVY2 .* dfz) .* t.LVY .* LPMuY + Svygamma;
        
        % (4.E30) No pressure terms
        Kygamma0    = Fz .* (t.PKY6 + t.PKY7 .* dfz) * t.LKYC;
        % (4.E30) With pressure terms
        % Kygamma0    = Fz .* (t.PKY6 + t.PKY7 .* dfz) * (1 + t.PPY5 * dpi) * t.LKYC;
        
        % (4.E25) No pressure terms
        Kyalpha     = t.PKY1 * Fzp .* (1 - t.PKY3 * abs(gammaStar)) .* sin(t.PKY4 * atan((Fz ./ Fzp) ./ ((t.PKY2 + t.PKY5 * gammaStar .^ 2)))) * t.LKY;
        % (4.E25) With pressure terms
        % Kyalpha     = t.PKY1 * Fzp * (1 + t.PPY1 * dpi) .* (1 - t.PKY3 * abs(gammaStar)) .* sin(t.PKY4 * atan((Fz ./ Fzp) ./ ((t.PKY2 + t.PKY5 * gammaStar .^ 2) * (1 + t.PPY2 * dpi))));
        
        % (4.E26)
        By          = Kyalpha / (Cy * Dy + Eps);
        
        % (4.E27)
        Shy         = (t.PHY1 + t.PHY2 .* dfz) * t.LHY + ((Kygamma0 .* gammaStar - Svygamma)./(Kyalpha + Eps)); % + params
        
        % (4.E20)
        alphay      = alphaStar' + Shy;
        
        % (4.E24)
        Ey          = (t.PEY1 + t.PEY2 .* dfz) .* (1 + t.PEY5 .* gammaStar .^ 2 - (t.PEY3 + t.PEY4 .* gammaStar) .* sign(alphay)) * t.LEY;
        
        % (4.E19)
        Fy          = (Dy .* sin(Cy .* atan((By .* alphay - Ey .* (By .* alphay - atan(By .* alphay))))) + Svy);
        
        result      = Fy;
        
    case {'fc'}
        % Combined load not yet implemented
        F           = [NaN, NaN];
        result      = F;
        
    case {'py'}
        % Peak Fy -> SA finder
        options = optimset('Display', 'none', 'TolX', .1, 'TolFun', .5, 'MaxIter', maxItr);
        
%         numPoints = length(Fz);
%         for i = 1 : numPoints
%             if (length(SA) < numPoints); SA(1 : numPoints) = SA(1); end
%             if (length(SR) < numPoints); SR(1 : numPoints) = SR(1); end
%             if (length(IA) < numPoints); IA(1 : numPoints) = IA(1); end
%             if (length(V) < numPoints); V(1 : numPoints) = V(1); end
%             
%             fy = @(sa) sign(sa) .* Pacejka(t, 'fy', Fz(i), sa, SR(i), IA(i), mu, V(i), 0);
%             
%             [sa,F,exitflag,output] = fminsearch(fy, sign(SA(i)) * (t.ALPMAX * 180 / pi()), options);
%             
%             result(i,:) = [sa, F];
%         end

        for i = length(Fz)
            fy = @(sa) sign(sa) * Pacejka(t, 'fy', Fz(i), sa, SR, IA, mu, V, 0);
            
            [sa, F, exitflag, output] = fminsearch(fy, sign(SA) * ((t.ALPMAX * 180 / pi()) / 2), options);
            
%             [sa, F]
            
            if abs(sa) > (peakLimit * (t.ALPMAX * 180 / pi()))
                warning(['Peak SA Value Exceedes ', num2str(peakLimit), 'x the Tested Range'])
                sa = sign(sa) * (peakLimit * (t.ALPMAX * 180 / pi()));
                F = Pacejka(t, 'fy', Fz(i), sa, SR, IA, mu, V, 0);
                result(i,:) = [sa, F];
            else
                result(i,:) = [sa, sign(SA) * F];
            end
        end
        
    case {'px'}
        % Peak Fx finder
        options = optimset('Display', 'none', 'TolX', .01, 'TolFun', .5, 'MaxIter', maxItr);
        
        for i = length(Fz)
            fx = @(sr) -sign(sr) * Pacejka(t, 'fx', Fz(i), SA, sr, IA, mu, V, 0);
            
            [sr, F, exitflag, output] = fminsearch(fx, sign(SR) * (t.KPUMAX / 2), options);
           
            if abs(sr) > (peakLimit * t.KPUMAX)
                warning(['Peak SR Value Exceedes ', num2str(peakLimit), 'x the Tested Range'])
                sr = sign(sr) * (peakLimit * t.KPUMAX);
                F = Pacejka(t, 'fx', Fz(i), SA, sr, IA, mu, V, 0);
                result(i,:) = [sr, F];
            else
                result(i,:) = [sr, -sign(sr) * F];
            end
        end
        
    case {'ry'}
        % Reverse Fy to SA lookup
        testPeak = Pacejka('py',Fz,SA,SR,IA,mu,V,0);
        if(max(abs(tF)) > abs(testPeak))
            warning('No solution exists at given load conditions')
            result = NaN;
        else
            options = optimset('Display','none','TolX',.05,'TolFun',.5,'MaxIter',maxItr);
            for i = 1:length(tF)
                [sa,F,exitflag,output] = fzero(@(SA) (Pacejka(t,'fy',Fz,SA,SR,IA,mu,V,0) - tF(i)),5 * sign(SA),options);
                result(i) = sa;
                disp(exitflag)
                disp(output)
            end
        end
        
    case {'rx'}
        % Reverse Fy to SA lookup
        testPeak = Pacejka('px',Fz,SA,SR,IA,mu,V,0);
        if(max(abs(tF)) > abs(testPeak))
            warning('No solution exists at given load conditions')
            result = NaN;
        else
            options = optimset('Display','none','TolX',.1,'TolFun',.5,'MaxIter',maxItr);
            for i = 1:length(tF)
                [sr,F,exitflag,output] = fzero(@(SR) (Pacejka(t,'fx',Fz,SA,SR,IA,mu,V,0) - tF(i)),-.1,options);
                result(i) = sr;
                disp(exitflag)
                disp(output)
            end
        end
        
    otherwise
        error('Incorrect Input Type. Valid options are Fy, Fx, or Fc')
        F           = [NaN, NaN];
        result      = F;
end

end