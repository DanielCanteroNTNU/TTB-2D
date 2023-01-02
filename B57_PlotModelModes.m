function [] = B57_PlotModelModes(Model,Calc)

% Plots the coupled system modes of vibration

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
% Calc = Structure with Calc variables, including at least:
%   .Plot.Model_modes = Array of modes to plot, eg: [1,4] plots modes 1 and 4
% ---- Output ----
% -------------------------------------------------------------------------

n = length(Calc.Plot.Model_modes);
if n > 0

    modes = Calc.Plot.Model_modes;
    max_subs = 4;
    if n<max_subs
        max_subs = n;
    end % if n<max_subs

    % Mode shapes plotting
    h_rail = 1; h_sleepers = 0.5; counter = 0;
    for mode = modes
        counter = counter + 1;
        if mod(mode,max_subs) == 1
            figure;
            counter = 1;
        end % if mod(mode,max_subs) == 1
        ind_vert = [Model.Mesh.DOF.rail_vert,Model.Mesh.DOF.sleepers,Model.Mesh.DOF.ballast_app,...
            Model.Mesh.DOF.beam_vert,Model.Mesh.DOF.ballast_aft];
        norm_factor = max(abs(Model.Modal.modes(ind_vert,mode)));
        subplot(max_subs,1,counter); hold on; box on;
            xdata = Model.Mesh.XLoc.rail_vert;
            plot(xdata,Model.Modal.modes(Model.Mesh.DOF.rail_vert,mode)/norm_factor+h_rail,'b');
            xdata = Model.Mesh.XLoc.sleepers;
            plot(xdata,Model.Modal.modes(Model.Mesh.DOF.sleepers,mode)/norm_factor+h_sleepers,'g.');
            xdata = Model.Mesh.XLoc.ballast_app;
            plot(xdata,Model.Modal.modes(Model.Mesh.DOF.ballast_app,mode)/norm_factor,'r.');
            xdata = Model.Mesh.XLoc.beam_vert;
            plot(xdata,Model.Modal.modes(Model.Mesh.DOF.beam_vert,mode)/norm_factor,'m');
            xdata = Model.Mesh.XLoc.ballast_aft;
            plot(xdata,Model.Modal.modes(Model.Mesh.DOF.ballast_aft,mode)/norm_factor,'r.');
        axis tight;
        if mod(mode,max_subs) == 0
            xlabel('Distance (m)');
        end % if mod(mode,max_subs) == 0
        ylabel({'Normalized';'mode'});
        title(['Mode ',num2str(mode),' (',num2str(round(Model.Modal.f(mode),3)),' Hz)']);

    end % for mode

end % if n > 0

% ---- End of function ----
