function [] = C01_ModelDeformationPlot(Calc,Model,Sol,Train)

% Plotting the model results in 3 subplots showing:
%       1) Rail vertical results
%       2) Sleepers vertical results
%       4) Ballast + Bridge vertical results
% The user has the option to change the show time step by incresing in:
%       -10%
%       -1%
%       +1%
%       +10%
% Also the magnitude to plot can be chosen:
%       Displacements
%       Dynamic displacements (Total - Static)
%       Velocities
%       Accelerations
% Press the button "Close" to close figure.

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
% % ---- Inputs ----
% Calc = Structure with Calc variables, including at least:
%   ... see script ...
% Model = Structure with Model variables, including at least:
%   ... see script ...
% Sol = Structure with Sol variables, including at least:
%   ... see script ...
% Train = Structure with Train variables, including at least:
%   ... see script ...
% % ---- Outputs ----
% -------------------------------------------------------------------------

set(0,'Units','pixels');

% Auxiliary variables
k_continue = 1;
t_per = 50;
t_per_add = 0;
data2plot = 1;

figure; 

disp('Select time increments with input Menu ...')

while k_continue == 1

    % Selection of time-step to plot
    t_per = t_per + t_per_add;
    
    % Checking selected time-step
    if t_per < 0
        t_per = 0;
    elseif t_per > 100
        t_per = 100;
    end % if t_per < 0

    % Data selection to plot
    switch data2plot
        case 1
            Data = Sol.Model.Nodal.U*1000;
            Data_text = 'Disp (mm)';
        case 2
            Data = (Sol.Model.Nodal.U - Sol.Model.Nodal.StaticU)*1000;
            Data_text = 'Dyn. Disp (mm)';
        case 3
            Data = Sol.Model.Nodal.V;
            Data_text = 'Velocity (m/s)';
        case 4
            Data = Sol.Model.Nodal.A;
            Data_text = 'Acc. (m/s^2)';
    end % switch data2plot

    % ---- Model deformation at one particular time instant ----
    ylim_add_per = 10;              % Margins addition on Y axis in percentage [%]
    t = round(Calc.Solver.num_t*t_per/100)+1-ceil(t_per/100);
    subplot(3,1,1); hold on; box on;
        plot(Model.Mesh.XLoc.rail_vert,Data(Model.Mesh.DOF.rail_vert,t));
        axis tight; Ylim = ylim; Ylim = Ylim + sign(Ylim).*diff(Ylim)*ylim_add_per/100/2; ylim(Ylim);
        for veh_num = 1:Train.Veh(1).Tnum
            plot([1;1]*Calc.Veh(veh_num).x_path(:,t)',Ylim'*ones(1,Train.Veh(veh_num).Wheels.num),'k:');
        end % for veh_num
        ylabel([{'Rail'},{['Vert. ',Data_text]}]);
        title(['Model vertical displacements for t = ',num2str(round(Calc.Solver.t(t),2)),'s']);
    subplot(3,1,2); hold on; box on;
        plot(Model.Mesh.XLoc.sleepers,Data(Model.Mesh.DOF.sleepers,t),'.');
        axis tight; Ylim = ylim; Ylim = Ylim + sign(Ylim).*diff(Ylim)*ylim_add_per/100/2; ylim(Ylim);
        for veh_num = 1:Train.Veh(1).Tnum
            plot([1;1]*Calc.Veh(veh_num).x_path(:,t)',Ylim'*ones(1,Train.Veh(veh_num).Wheels.num),'k:');
        end % for veh_num
        ylabel([{'Sleepers'},{['Vert. ',Data_text]}]);
    subplot(3,1,3); hold on; box on;
        plot(Model.Mesh.XLoc.ballast_app,Data(Model.Mesh.DOF.ballast_app,t),'b.');
        plot(Model.Mesh.XLoc.beam_vert,Data(Model.Mesh.DOF.beam_vert,t),'b');
        plot(Model.Mesh.XLoc.ballast_aft,Data(Model.Mesh.DOF.ballast_aft,t),'b.');
        axis tight; Ylim = ylim; Ylim = Ylim + sign(Ylim).*diff(Ylim)*ylim_add_per/100/2; ylim(Ylim);
        for veh_num = 1:Train.Veh(1).Tnum
            plot([1;1]*Calc.Veh(veh_num).x_path(:,t)',Ylim'*ones(1,Train.Veh(veh_num).Wheels.num),'k:');
        end % for veh_num
        ylabel([{'Ballast and Beam'},{['Vert. ',Data_text]}]);
        xlabel('Distance (m)');

    % ---- Action Menu ----    
    choice = menu('Action','t = 0','t = -10%','t = -1%','t = -0.1%', ...
        't = +0.1%','t = +1%','t = +10%','t = end','Show: U', ...
        'Show: Dyn. U','Show: V','Show: Acc.','Close');
    switch choice
        case 1
            t_per = 0;
            t_per_add = 0;
        case 2
            t_per_add = -10;
        case 3
            t_per_add = -1;
        case 4
            t_per_add = -0.1;
        case 5
            t_per_add = +0.1;
        case 6
            t_per_add = +1;
        case 7
            t_per_add = +10;
        case 8
            t_per = 100;
            t_per_add = 0;
        case 9
            data2plot = 1;  % Displacements
            t_per_add = 0;
        case 10
            data2plot = 2;  % Dynamic displacements
            t_per_add = 0;
        case 11
            data2plot = 3;  % Velocity
            t_per_add = 0;
        case 12
            data2plot = 4;  % Acceleration
            t_per_add = 0;
        case 13
            k_continue = 0;
    end % switch choice

    % Clear current figure
    clf;

end % if Calc.Plot.Model.P01_ModelDef >=0
close

% ---- End of script ----
