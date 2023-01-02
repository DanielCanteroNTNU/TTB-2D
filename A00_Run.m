% Main script to run TTB-2D model

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

clear; clc;

% --------------------------- Definitions ---------------------------------

% ---- Train ----
A01_Train;

% ---- Track ----
A02_Track;

% ---- Bridge ----
A03_Bridge;

% ---- Options ----
A04_Options;

% --------------------------- Calculations --------------------------------

B00_Calculations;

%% -------------------- Results processing and plotting --------------------

% % Model deformation plot at a given instant in time
% C01_ModelDeformationPlot(Calc,Model,Sol,Train);

% % Time history plot of user selected model point
% C02_TimeHistoryPlot(Model,Sol,Beam,Calc);

% Other user-specified plots of the results
C03_TTB_2D_Plots;

% ---- End of script ----
