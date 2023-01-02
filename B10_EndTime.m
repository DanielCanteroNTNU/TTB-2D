function [Calc] = B10_EndTime(Calc)

% Calculates the time t_end, when the load reaches the end of the path
%   and generetes other position, velocities and acceleration variables.

% *************************************************************************
% *** Script part of TTB-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***% ***                                                                   ***
% *** If you found this tool useful, please cite:                       ***
% *** D. Cantero. TTB-2D: Train-Track-Bridge interaction simulation tool***
% ***   for Matlab, SoftwareX, Volume 20, 2022.                         ***
% ***   DOI: https://doi.org/10.1016/j.softx.2022.101253                ***
% ***                                                                   ***
% *************************************************************************

% -------------------------------------------------------------------------
% ---- Input ----
% Calc = Structure with Calculation variables, including at least:
%   .Position.x_0 = X coordinate of moving load starting point
%   .Position.x_end = X coordinate of moving load end
%   .Position.v_0 = Initial velocity of moving load
%   .Position.a_0 = Initial acceleration of moving load
%   .Position.aa = Exponent of acceleration
%   .Cte.tol = Treshold under which two numerical values are considered identical
% ---- Output ----
% Calc = Additional fields in the structure:
%   .Time.t_end = Time it takes the load to reach Calc.Position.x_end
%   .Position.v_end = Velocity at Calc.Time.t_end
%   .Position.a_end = Acceleration at Calc.Time.t_end
%   .Position.v_max = Maximum velocity reach by the moving load
%   .Position.v_min = Minimum velocity reach by the moving load
%   .Position.a_max = Maximum acceleration reach by the moving load
%   .Position.a_min = Minimum velocity reach by the moving load
% -------------------------------------------------------------------------

dt = 1;     % Starting time incremental step

% Auxiliary varibles
L = Calc.Position.x_end-Calc.Position.x_0;
k_cont = 1;
counter = 0;
max_counter = 100 + L/dt;
t1 = 0;

% Solution search
while k_cont == 1
    t = [t1,t1+dt];

    counter = counter + 1;
    
    Lt = Calc.Position.v_0*t + Calc.Position.a_0*t.^Calc.Position.aa;

    if sum(sign(L - Lt)) == 0
        if dt > Calc.Cte.tol
            dt = dt/2;
        else
            k_cont = 0;
        end % if dt
    elseif sum(sign(L - Lt)) == 1
        t = t(2)*[1,1];
        k_cont = 0;
    else
        t1 = t1 + dt;
    end % end if sum

    if counter >= max_counter
        disp('Initial position / Velocity / Acceleration are WRONG!!!');
        error('Initial position / Velocity / Acceleration are WRONG!!!');
    end
end % while k_cont

% -- Output calculation and generation --
Calc.Time.t_end = mean(t);

Calc.Position.v_end = Calc.Position.v_0 + ...
    Calc.Position.aa*Calc.Position.a_0*Calc.Time.t_end^(Calc.Position.aa-1);
Calc.Position.a_end = Calc.Position.aa* ...
    (Calc.Position.aa-1)*Calc.Position.a_0*Calc.Time.t_end^(Calc.Position.aa-2);

Calc.Position.v_max = max(Calc.Position.v_0,Calc.Position.v_end);
Calc.Position.v_min = min(Calc.Position.v_0,Calc.Position.v_end);
Calc.Position.a_max = max(Calc.Position.a_0,Calc.Position.a_end);
Calc.Position.a_min = min(Calc.Position.a_0,Calc.Position.a_end);

% ---- End of function ----
