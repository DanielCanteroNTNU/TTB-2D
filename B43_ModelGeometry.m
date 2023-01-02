function [Calc,Train,Beam] = B43_ModelGeometry(Calc,Train,Track,Beam)

% Auxiliary calculations related to the model geometry

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
% Calc = Structure with Calc's variables, including at least:
%   ... see script ...
% Train = Structure with Train variables, including at least:
%	... see script ...
% Track = Structure with Track variables, including at least:
%	... see script ...
% Beam = Structure with Beam variables, including at least:
%	... see script ...
% % ---- Outputs ----
% Calc = Structure with Calc variables and new fields:
%   ... see script ...
% Train = Structure with Train variables and new fields:
%   ... see script ...
% Beam = Structure with Beam variables and new fields:
%   ... see script ...
% -------------------------------------------------------------------------

% -- Track length --

% Model redux option default value
if ~isfield(Calc.Options,'redux')
    Calc.Options.redux = 1;
end % if ~isfield(Calc.Options,'redux')

% Redux_factor (Is 1 if redux is Off. Gives 0 if redux is ON)
Calc.Options.redux_factor = (Calc.Options.redux==0);

% Approach distance (distance from first front wheel to bridge start)
Calc.Profile.L_Approach = ...
    round(Calc.Profile.minL_Approach/Track.Sleeper.spacing)*Track.Sleeper.spacing;

% After crossing default value
if ~isfield(Calc.Profile,'minL_After')
    Calc.Profile.minL_After = 10;
end % if ~isfield(Calc.Profile,'minL_After')

% After crossing distance
Calc.Profile.L_After = ...
    round(Calc.Profile.minL_After/Track.Sleeper.spacing)*Track.Sleeper.spacing;

% -- Beam model --

% Beam elements number and size
Beam.Mesh.Ele.L = (Track.Sleeper.spacing/Beam.Mesh.Ele.num_per_spacing);
Beam.Mesh.Ele.num = round(Beam.Prop.L/Beam.Mesh.Ele.L);

% Modified beam length
Beam.Prop.L = Beam.Mesh.Ele.L*Beam.Mesh.Ele.num;
Calc.Profile.L_bridge = Beam.Prop.L;

% Cross-section Area (Not needed, because mass is defined in terms of rho)
Beam.Prop.A = 1;

% -- Vehicle variables --

% Total number of vehicles
Train.Veh(1).Tnum = size(Train.Veh,2);

% Total train length and approach distances for each vehicle
Train.Veh(1).TL = Train.Veh(1).Body.L + Train.Veh(1).Bogie.L(1)/2;
Train.Veh(1).L_app = Calc.Profile.L_Approach;
for veh_num = 2:Train.Veh(1).Tnum
    % Total length of vehicle
    Train.Veh(1).TL = Train.Veh(1).TL + Train.Veh(veh_num-1).Body.Le(2) + ...
        Train.Veh(veh_num).Body.Le(1) + Train.Veh(veh_num).Body.L;
    % Approach distances
    Train.Veh(veh_num).L_app = Train.Veh(veh_num-1).L_app + ...
        Train.Veh(veh_num-1).Bogie.L(1)/2 + Train.Veh(veh_num-1).Body.L + ...
        Train.Veh(veh_num-1).Body.Le(2) + Train.Veh(veh_num).Body.Le(1) - ...
        Train.Veh(veh_num).Bogie.L(1)/2;
end % for veh_num = 2:Train.Veh(1).Tnum
Train.Veh(1).TL = Train.Veh(1).TL + Train.Veh(end).Bogie.L(2)/2;

% Geometric properties for each vehicle
for veh_num = 1:Train.Veh(1).Tnum
    % Axle distances
    Train.Veh(veh_num).Ax_dist = [0,Train.Veh(veh_num).Bogie.L(1),...
        Train.Veh(veh_num).Bogie.L(1)/2 + Train.Veh(veh_num).Body.L + ...
        [-1,1]*Train.Veh(veh_num).Bogie.L(2)/2];
    % Wheelbase
    Train.Veh(veh_num).wheelbase = Train.Veh(veh_num).Ax_dist(end);
    % First wheel distance to Locomotive's first wheel
    Train.Veh(veh_num).First_wheel_dist = ...
        Train.Veh(veh_num).L_app - Train.Veh(1).L_app;
    % Vehicle contact force flag
    Calc.Train.Veh(veh_num).Positive_ContactForce = 0;
end % for veh_num = 1:Train.Veh(1).Tnum

% Vehicle total length rounded up to closest number of sleeper spacings
Train.Veh(1).max_TL = ...
    ceil(Train.Veh(1).TL/Track.Sleeper.spacing)*Track.Sleeper.spacing;
Calc.Profile.max_TL = Train.Veh(1).max_TL;

% Additional model length to have start and end the model with a sleeper.
Calc.Profile.extra_L = mod(Beam.Prop.L,Track.Sleeper.spacing);
if Calc.Profile.extra_L > 0
    Calc.Profile.extra_L = Track.Sleeper.spacing - Calc.Profile.extra_L;
end % if Calc.Profile.extra_L > 0

% Additional number of sleepers behind the vehicle at start and end
Track.Rail.Options.num_add_sleepers = 10;

% Additional length not needed when working with the redux model
if Calc.Options.redux == 1
    Track.Rail.Options.num_add_sleepers = 0;
end % if Calc.Options.redux == 1

% Additional length to remove the influence of the track ends
Calc.Profile.extra_L2 = Track.Rail.Options.num_add_sleepers*Track.Sleeper.spacing;

% Minimum total length of profile
Calc.Profile.L = 2*Calc.Profile.max_TL*Calc.Options.redux_factor ...
    + Train.Veh(1).L_app + Beam.Prop.L + Calc.Profile.extra_L + ...
    2*Calc.Profile.extra_L2*Calc.Options.redux_factor + Calc.Profile.L_After;
Calc.Profile.L = round(Calc.Profile.L/Track.Sleeper.spacing)*Track.Sleeper.spacing;

% Vehicle's initial position
Calc.Position.x_start_end = ...
    [(Train.Veh(1).max_TL + Calc.Profile.extra_L2)*Calc.Options.redux_factor,...
    Calc.Profile.L - Calc.Profile.extra_L2*Calc.Options.redux_factor + ...
    Train.Veh(1).max_TL*(1-Calc.Options.redux_factor)];

% Length of approach + Train total length (Used for plotting)
Calc.Profile.L_Aw = Calc.Profile.L_Approach + Calc.Profile.max_TL*Calc.Options.redux_factor;

% ---- End of function ----
