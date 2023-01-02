function [Calc] = B11_TimeSpaceDiscretization(Calc)

% Generates the uniformly spaced time discretization, 
%   and correponding space discretization.

% *************************************************************************
% *** Script part of TTB-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% ***                                                                   ***
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
%   .Time.t_end = Time it takes the load to reach Calc.Position.x_end
%   .Position.a_max = Maximum acceleration reach by the moving load
%   .Solver.t_steps_per_second = Steps per second to be used in the time discretization
%   .Cte.tol = Treshold under which two numerical values are considered identical
%   .Plot.P3_VehPos = If it exists, a figure is shown with 3 subplots:
%       1) Load position in time
%       2) Load velocity in time
%       3) Load acceleration in time
% ---- Output ----
% Calc = Additional fields in the structure:
%   .Solver.t = Array containing the discretized time values
%   .Solver.dt = Time step
%   .Solver.num_t = Total number of time steps
%   .Time.t_0_ind = Index of Calc.x where t = 0
%   .Position.x = Discretization of X dimension
%   .Position.num_x = Total number of space discretization points
%   .Time.t_end_ind = Index of Calc.x where t = Calc.Time.t_end
% -------------------------------------------------------------------------

% Calculates the appropriate time step length for the problem cosidering
% the maximum frequency of interest
Calc.Solver.t_steps_per_second = Calc.Solver.max_accurate_frq*2;

% Time solver array
Calc.Solver.t = linspace(0,Calc.Time.t_end,ceil(Calc.Solver.t_steps_per_second*Calc.Time.t_end)+1);

% Solver sampling time 
Calc.Solver.dt = Calc.Solver.t(2)-Calc.Solver.t(1);

% Number of solver steps
Calc.Solver.num_t = length(Calc.Solver.t);

% Based on start and end position of vehicle 
% calculation of corresponding start/end time indices
aux1 = Calc.Position.v_0*Calc.Solver.t + Calc.Position.a_0*Calc.Solver.t.^Calc.Position.aa;
dx_start = abs(diff(aux1));
dx_end = dx_start(end);
dx_start = dx_start(1);
Calc.Time.t_0_ind = 1;

if Calc.Position.x_0 > 0
    aux2 = linspace(0,Calc.Position.x_0,round(Calc.Position.x_0/dx_start)+1);
    Calc.Time.t_0_ind = length(aux2);
    aux1 = [aux2(1:end-1),aux1+Calc.Position.x_0];
end % if Calc.Position.x_0

if Calc.Position.x_end < Calc.Profile.L
    aux2 = linspace(Calc.Position.x_end,Calc.Profile.L,round((Calc.Profile.L-Calc.Position.x_end)/dx_end));
    aux1 = [aux1(1:end-1),aux2];
end % for Calc.Position.x_end

Calc.Position.x = aux1;
Calc.Position.num_x = length(Calc.Position.x);
Calc.Time.t_end_ind = Calc.Time.t_0_ind+Calc.Solver.num_t-1;

% ---- Plotting Results ----
if Calc.Plot.P3_VehPos == 1
    figure;
    subplot(3,1,1); plot(Calc.Position.x(Calc.Time.t_0_ind:Calc.Time.t_end_ind),Calc.Solver.t);
        xlabel('Vehicle Position (m) with respect to the Profile'); ylabel('Time (s)'); 
        %xlim([0,Calc.Profile.L]); ylim([0,Calc.Time.t_end]);
        xlim([0,Calc.Position.x_end]); ylim([0,Calc.Time.t_end]);
        hold on; plot(Calc.Profile.L*[1,1],ylim,'k--');
        text(Calc.Profile.L,ylim*[0;1],'End of profile','HorizontalAlignment','Right','VerticalAlignment','Top');
        title('FRONT wheel position, velocity and Acceleration')
    subplot(3,1,2); plot(Calc.Solver.t,Calc.Position.v_0+Calc.Position.aa*Calc.Position.a_0*Calc.Solver.t.^(Calc.Position.aa-1));
        xlabel('Time(s)'); ylabel('Velocity (m/s)'); xlim([0,Calc.Time.t_end]); ylim([0,Calc.Position.v_max]);
        if all([Calc.Position.a_min,Calc.Position.a_max] == 0); ylim([0,Calc.Position.v_max*1.2]); end
    subplot(3,1,3); plot(Calc.Solver.t,Calc.Position.aa*(Calc.Position.aa-1)*Calc.Position.a_0*Calc.Solver.t.^(Calc.Position.aa-2));
        xlabel('Time(s)'); ylabel('Acceleration (m^2/s)'); xlim([0,Calc.Time.t_end]); 
        try % In case both a_min and a_max are zero
            ylim([min(0,Calc.Position.a_min),max(0,Calc.Position.a_max)]*1.2);
        end % try
end % if Calc.Plot.P3_VehPos == 1

% ---- End of function ----
