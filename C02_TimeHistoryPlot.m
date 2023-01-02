function [] = C02_TimeHistoryPlot(Model,Sol,Beam,Calc)

% Plotting time history results of particular nodes.
% There are 3 subplots showing:
%       1) Overview of model nodes, and node under consideration
%       2) Veritical displacements of selected node
%       4) Rotational displacement of selected node (only applicable in
%           rail and beam nodes)
% The user has the option to change the node numer by incresing in:
%       -100 nodes
%       -10 nodes
%       -1 nodes
%       +1 nodes
%       +10 nodes
%       +100 nodes
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
% Model = Structure with Model variables, including at least:
%   ... see script ...
% Sol = Structure with Sol variables, including at least:
%   ... see script ...
% Beam = Structure with Beam variables, including at least:
%   ... see script ...
% Calc = Structure with Calc variables, including at least:
%   ... see script ...
% % ---- Outputs ----
% -------------------------------------------------------------------------

set(0,'Units','pixels');

% Auxiliary variables
k_continue = 1;
node_num = length(Model.Mesh.DOF.rail_vert) + length(Model.Mesh.DOF.sleepers) + ...
    length(Model.Mesh.DOF.ballast_app) + floor(length(Model.Mesh.DOF.beam_vert)/2);
node_num_add = 0;
data2plot = 1;
MarkerSize = 4;
h4sub1 = [2,1,0];

DOF = [Model.Mesh.DOF.rail_vert, ...
    Model.Mesh.DOF.sleepers, ...
    Model.Mesh.DOF.ballast_app, ...
    Model.Mesh.DOF.beam_vert, ...
    Model.Mesh.DOF.ballast_aft];
DOF1 = 1;
DOF1 = [DOF1,length(Model.Mesh.DOF.rail_vert)+DOF1(1)];
DOF1 = [DOF1,length(Model.Mesh.DOF.sleepers)+DOF1(2)];
DOF1 = [DOF1,length(Model.Mesh.DOF.ballast_app)+DOF1(3)];
DOF1 = [DOF1,length(Model.Mesh.DOF.beam_vert)+DOF1(4)];
DOF_x = [Model.Mesh.XLoc.rail_vert, ...
    Model.Mesh.XLoc.sleepers, ...
    Model.Mesh.XLoc.ballast_app, ...
    Model.Mesh.XLoc.beam_vert, ...
    Model.Mesh.XLoc.ballast_aft];
DOF_y = [h4sub1(1)*ones(size(Model.Mesh.XLoc.rail_vert)), ...
    h4sub1(2)*ones(size(Model.Mesh.XLoc.sleepers)), ...
    h4sub1(3)*ones(size(Model.Mesh.XLoc.ballast_app)), ...
    h4sub1(3)*ones(size(Model.Mesh.XLoc.beam_vert)), ...
    h4sub1(3)*ones(size(Model.Mesh.XLoc.ballast_aft))];

Tnum_nodes = length(DOF);

figure; 

disp('Select time increments with input Menu ...')

while k_continue == 1

    % Selection of node
    node_num = node_num + node_num_add;

    % Checking selected node number
    if node_num < 1
        node_num = 1;
    elseif node_num > Tnum_nodes
        node_num = Tnum_nodes;
    end % if node_num < 1
    
    % Plot rotations
    if node_num < DOF1(2)
        plot_rot = 1;
    elseif and(node_num >= DOF1(4),node_num < DOF1(5))
        plot_rot = 1;
    else
        plot_rot = 0;
    end % if node_num < DOF1(2)
    
    % Data selection to plot
    switch data2plot
        case 1
            Data = Sol.Model.Nodal.U;
            Data_text_sub1 = 'Disp (mm)'; Data_factor_sub1 = 1000;
            Data_text_sub2 = 'Rot (rad)'; Data_factor_sub2 = 1;
        case 2
            Data = (Sol.Model.Nodal.U - Sol.Model.Nodal.StaticU);
            Data_text_sub1 = 'Dyn. Disp (mm)'; Data_factor_sub1 = 1000;
            Data_text_sub2 = 'Dyn. Rot (rad)'; Data_factor_sub2 = 1;
        case 3
            Data = Sol.Model.Nodal.V;
            Data_text_sub1 = 'Velocity (m/s)'; Data_factor_sub1 = 1;
            Data_text_sub2 = 'Velocity of Rot'; Data_factor_sub2 = 1;
        case 4
            Data = Sol.Model.Nodal.A;
            Data_text_sub1 = 'Acc. (m/s^2)'; Data_factor_sub1 = 1;
            Data_text_sub2 = 'Acc. of Rot'; Data_factor_sub2 = 1;
    end % switch data2plot

    % ---- Model Nodes ----
    subplot(3,1,1); hold on; box on;
        plot(Model.Mesh.XLoc.rail_vert,h4sub1(1)*ones(size(Model.Mesh.XLoc.rail_vert)),'k.','MarkerSize',MarkerSize);
        plot(Model.Mesh.XLoc.sleepers,h4sub1(2)*ones(size(Model.Mesh.XLoc.sleepers)),'k.','MarkerSize',MarkerSize);
        plot(Model.Mesh.XLoc.ballast_app,h4sub1(3)*ones(size(Model.Mesh.XLoc.ballast_app)),'k.','MarkerSize',MarkerSize);
        plot(Model.Mesh.XLoc.beam_vert,h4sub1(3)*ones(size(Model.Mesh.XLoc.beam_vert)),'k.','MarkerSize',MarkerSize);
        aux1 = 1:Beam.Mesh.Ele.num_per_spacing:length(Model.Mesh.XLoc.beam_vert);
        plot(Model.Mesh.XLoc.beam_vert(aux1),h4sub1(3)*ones(size(Model.Mesh.XLoc.beam_vert(aux1))),'k.','MarkerSize',MarkerSize);
        plot(Model.Mesh.XLoc.ballast_aft,h4sub1(3)*ones(size(Model.Mesh.XLoc.ballast_aft)),'k.','MarkerSize',MarkerSize);
        ylim([h4sub1(3)-1,h4sub1(1)+1]);
        text(diff(Model.Mesh.XLoc.rail_vert([1,end]))*0.05,h4sub1(1)+0.5,'Rail');
        text(diff(Model.Mesh.XLoc.rail_vert([1,end]))*0.15,h4sub1(2)+0.5,'Sleeper');
        text(diff(Model.Mesh.XLoc.rail_vert([1,end]))*0.25,h4sub1(3)+0.5,'Ballast');
        text(Model.Mesh.XLoc.beam_vert(1),h4sub1(3)+0.5,'Beam');
        set(gca,'YTick',[]);
        xlim(Model.Mesh.XLoc.rail_vert([1,end]));
        xlabel('Distance');
        plot(Model.Mesh.XLoc.beam_vert(1)*[1,1],ylim,'k--');
        plot(Model.Mesh.XLoc.beam_vert(end)*[1,1],ylim,'k--');
        plot(DOF_x(node_num),DOF_y(node_num),'ro');

    subplot(3,1,2); hold on; box on;
        plot(Calc.Solver.t,Data(DOF(node_num),:)*Data_factor_sub1); axis tight;
        xlabel('Time (s)');
        ylabel([{'Vert. Disp.'},{Data_text_sub1}]);

    if plot_rot == 1
        subplot(3,1,3); hold on; box on;
            plot(Calc.Solver.t,Data(DOF(node_num)+1,:)*Data_factor_sub2); axis tight;
            xlabel('Time (s)');
            ylabel([{'Nodal rotation'},{Data_text_sub2}]);
    end % if plot_rot == 1

    % ---- Action Menu ----    
    choice = menu('Action','Part: Rail','Part: Sleeper','Part: Ballast',...
        'Part: Beam','Show: U','Show: Dyn. U','Show: V','Show: Acc.',...
        'Move: -100 Nodes','Move: -10 Nodes','Move: -1 Node',...
        'Move: +1 Node','Move: +10 Nodes','Move: +100 Nodes','Close');
    switch choice
        case 1
            field2plot = 'Rail';
            node_num = DOF1(1);
            node_num_add = 0;
        case 2
            field2plot = 'Sleeper';
            node_num = DOF1(2);
            node_num_add = 0;
            plot_rot = 0;
        case 3
            field2plot = 'Ballast';
            node_num = DOF1(3);
            node_num_add = 0;
            plot_rot = 0;
        case 4
            field2plot = 'Beam';
            node_num = DOF1(4);
            node_num_add = 0;
            plot_rot = 1;
        case 5
            data2plot = 1;  % Displacements
            node_num_add = 0;
        case 6
            data2plot = 2;  % Dynamic displacements
            node_num_add = 0;
        case 7
            data2plot = 3;  % Velocity
            node_num_add = 0;
        case 8
            data2plot = 4;  % Acceleration
            node_num_add = 0;
        case 9
            node_num_add = -100;
        case 10
            node_num_add = -10;
        case 11
            node_num_add = -1;
        case 12
            node_num_add = +1;
        case 13
            node_num_add = +10;
        case 14
            node_num_add = +100;
        case 15
            k_continue = 0;
    end % switch choice

    % Clear current figure
    clf;

end % if Calc.Plot.Model.P01_ModelDef >=0
close

% ---- End of script ----
