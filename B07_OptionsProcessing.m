function [Calc,Train,Track,Beam] = B07_OptionsProcessing(Calc,Train,Track,Beam)

% Processing of input variables, and generating new or changing auxiliary 
% variables accordingly.

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
%   ... Many ...
% Train = Structure with Train variables, including at least:
%   ... Many ...
% Track = Structure with Track variables, including at least:
%   ... Many ...
% Beam = Structure with Beam variables, including at least:
%   ... Many ...
% ---- Output ----
% Calc = Addition of fields to structure Calc:
%   ... Many ...
% Train = Addition of fields to structure Train:
%   ... Many ...
% Track = Addition of fields to structure Track:
%   ... Many ...
% Beam = Addition of fields to structure Beam:
%   ... Many ...
% -------------------------------------------------------------------------

% Constants
Calc.Cte.tol = 1e-6;            % Numerical tolerance
Calc.Cte.grav = -9.81;          % Gravity [m/s^2]

% ------------------------- Type of calculation ---------------------------
% Note: Only coupled system solution implemented in TTB-2D

% Type of calculation text label
if ~isfield(Calc,'Type')
    Calc.Type.short_text = 'COUP';
end % if ~isfield(Calc,'Type')

% Other variables
if strcmp(Calc.Type.short_text,'COUP')
    Calc.Type.id = 1;
    Calc.Type.long_text = 'Coupled system calculation';
end % if strcmp(Calc.Type.short_text,'COUP')

% ------------------------------- Options ---------------------------------

% Starting Options field
if ~isfield(Calc,'Options')
    Calc.Options = [];
end % if ~isfield(Calc,'Options')

% Beam sections to calculate results
if ~isfield(Calc.Options,'calc_beam_sections')
    Calc.Options.calc_beam_sections = [];
    Calc.Options.num_calc_beam_sections = 0;
else
    Calc.Options.num_calc_beam_sections = length(Calc.Options.calc_beam_sections);
end % if ~isfield(Calc.Options,'calc_beam_sections')

% Calculate model frequencies
if ~isfield(Calc.Options,'calc_model_frq')
    Calc.Options.calc_model_frq = 0;
end % if ~isfield(Calc.Options,'calc_model_frq')

% Calculated model modes
if ~isfield(Calc.Options,'calc_model_modes')
    Calc.Options.calc_model_modes = 0;
end % if ~isfield(Calc.Options,'calc_model_modes')

% VBI on/off option
if ~isfield(Calc.Options,'VBI')
    Calc.Options.VBI = 1;
end % if ~isfield(Calc.Options,'VBI')

% Frequency of progress display (in seconds)
if ~isfield(Calc.Options,'disp_every_t')
    Calc.Options.disp_every = 2;
end % if ~isfield(Calc.Options,'disp_every_t')

% -------------------------------- Solver ---------------------------------

% Starting Solver field
if ~isfield(Calc,'Solver')
    Calc.Solver.was_empty = 1;
end % if ~isfield(Calc,'Solver')

% Newmark-Beta scheme
if ~isfield(Calc.Solver,'NewMark_damp')
    Calc.Solver.NewMark_damp = 0; % Average acceleration method (Normal Newmark-beta)
end % if ~isfield(Calc.Solver,'NewMark_damp')

% Newmark scheme constants
if Calc.Solver.NewMark_damp == 0
    % Default = Average (or constant) acceleration method
    Calc.Solver.NewMark_delta = 0.5; 
    Calc.Solver.NewMark_beta = 0.25;
elseif Calc.Solver.NewMark_damp == 1
    % Damped Newmark-Beta scheme
    Calc.Solver.NewMark_delta = 0.6; 
    Calc.Solver.NewMark_beta = 0.3025;
end % if Calc.Solver.NewmMark_damp

% -------------------------------- Profile --------------------------------

% Starting Profile field
if ~isfield(Calc,'Profile')
    Calc.Profile = [];
end % if ~isfield(Calc,'Profile')

% Default Smooth profile
if ~isfield(Calc.Profile,'Type')
    Calc.Profile.Type = 0;
end % if ~isfield(Calc.Profile,'Type')

% Minimum Profile sampling period (m)
if ~isfield(Calc.Profile,'min_dx')
    Calc.Profile.min_dx = min([abs(diff([Train.Veh(:).Ax_dist])),0.01]);
end % if ~isfield(Calc.Profile,'min_dx')

% ------------------------------- Plotting --------------------------------

% If no plots are selected
if ~isfield(Calc,'Plot')
    Calc.Plot.NoPlot = 1;
end % if ~isfield(Calc,'Plot')
if ~isfield(Calc.Plot,'Veh')
    Calc.Plot.Veh.NoPlot = 1;
end % if ~isfield(Calc.Plot,'Veh')
if ~isfield(Calc.Plot,'Model')
    Calc.Plot.Model.NoPlot = 1;
end % if ~isfield(Calc.Plot,'Model')
if ~isfield(Calc.Plot,'Beam')
    Calc.Plot.Beam.NoPlot = 1;
end % if ~isfield(Calc.Plot,'Beam')

% Change Settings to generate P1 plot
if isfield(Calc.Plot,'P1_Beam_frq')
    if Calc.Plot.P1_Beam_frq == 1
        Calc.Options.calc_beam_frq = 1;
    end % if Calc.Plot.P1_Beam_frq == 1
else
    Calc.Plot.P1_Beam_frq = 0;
end % if isfield(Calc.Plot,'P1_Beam_frq')

if isfield(Calc.Plot,'P2_Beam_modes')
    if Calc.Plot.P2_Beam_modes >= 1
        Calc.Options.calc_beam_frq = 1;
        Calc.Options.calc_beam_modes = 1;
    end % Calc.Plot.P2_Beam_modes
else
    Calc.Plot.P2_Beam_modes = 0;
end % if isfield(Calc.Plot,'P2_Beam_frq')

if ~isfield(Calc.Plot,'P3_VehPos')
    Calc.Plot.P3_VehPos = 0;
end % if isfield(Calc.Plot,'P3_VehPos')

if ~isfield(Calc.Plot,'Profile_original')
    Calc.Plot.Profile_original = 0;
end % if ~isfield(Calc.Plot,'Profile_original')

if ~isfield(Calc.Plot,'Model_modes')
    Calc.Plot.Model_modes = [];
end % if ~isfield(Calc.Plot,'Model_modes')

% ---- Vehicle ----
if ~isfield(Calc.Plot.Veh,'P01_VertDisp')
    Calc.Plot.Veh.P01_VertDisp = 0;
end
if ~isfield(Calc.Plot.Veh,'P02_VertVel')
    Calc.Plot.Veh.P02_VertVel = 0;
end
if ~isfield(Calc.Plot.Veh,'P03_VertAcc')
    Calc.Plot.Veh.P03_VertAcc = 0;
end 
if ~isfield(Calc.Plot.Veh,'P04_ContactForce_t')
    Calc.Plot.Veh.P04_ContactForce_t = 0;
end
if ~isfield(Calc.Plot.Veh,'P05_ContactForce_x')
    Calc.Plot.Veh.P05_ContactForce_x = 0;
end

% ---- Model ----
if ~isfield(Calc.Plot.Model,'P00_ModelVisualization')
    Calc.Plot.Model.P00_ModelVisualization = 0;
end 
if ~isfield(Calc.Plot.Model,'P01_ModelDef')
    Calc.Plot.Model.P01_ModelDef = -1;
end 
if ~isfield(Calc.Plot.Model,'P02_ModelRot')
    Calc.Plot.Model.P02_ModelRot = -1;
end

% ---- Beam ----
if ~isfield(Calc.Plot.Beam,'P01_DispContour')
    Calc.Plot.Beam.P01_DispContour = 0;
end
if ~isfield(Calc.Plot.Beam,'P02_StaticDispContour')
    Calc.Plot.Beam.P02_StaticDispContour = 0;
end
if ~isfield(Calc.Plot.Beam,'P03_BMContour')
    Calc.Plot.Beam.P03_BMContour = 0;
end
if ~isfield(Calc.Plot.Beam,'P04_StaticBMContour')
    Calc.Plot.Beam.P04_StaticBMContour = 0;
end
if ~isfield(Calc.Plot.Beam,'P05_ShearContour')
    Calc.Plot.Beam.P05_ShearContour = 0;
end
if ~isfield(Calc.Plot.Beam,'P06_StaticShearContour')
    Calc.Plot.Beam.P06_StaticShearContour = 0;
end
if ~isfield(Calc.Plot.Beam,'P07_VertAccContour')
    Calc.Plot.Beam.P07_VertAccContour = 0;
end
if ~isfield(Calc.Plot.Beam,'P08_Sections_BeamVertDisp')
    Calc.Plot.Beam.P08_Sections_BeamVertDisp = 0;
end
if ~isfield(Calc.Plot.Beam,'P09_Sections_BeamBM')
    Calc.Plot.Beam.P09_Sections_BeamBM = 0;
end
if ~isfield(Calc.Plot.Beam,'P10_Sections_BeamShear')
    Calc.Plot.Beam.P10_Sections_BeamShear = 0; 
end 
if ~isfield(Calc.Plot.Beam,'P11_Sections_BeamAcc')
    Calc.Plot.Beam.P11_Sections_BeamAcc = 0;
end

% ------------------------------- Vehicle ---------------------------------

% Velocity and acceleration
Train.Veh(1).VelAcc = [Train.vel,0,2];      % [v0, a0, aa]

% Default calculation of Vehicle natural frequencies
if ~isfield(Calc.Options,'calc_veh_frq')
    Calc.Options.calc_veh_frq = 1;
end % if ~isfield(Calc.Options,'calc_veh_frq')

% Load position, velocity and acceleration
Calc.Position.x_0 = Calc.Position.x_start_end(1);
Calc.Position.x_end = Calc.Position.x_start_end(2);
Calc.Position.v_0 = Train.Veh(1).VelAcc(1);
Calc.Position.a_0 = Train.Veh(1).VelAcc(2);
Calc.Position.aa = Train.Veh(1).VelAcc(3);
if and(abs(Calc.Position.a_0) > 0, Calc.Position.aa < 2)
    disp('The acceleration exponent aa should be greater or equal to 2!!!');
end % if and(abs(Calc.Position.a_0) > 0, Calc.Position.aa < 2)

% -------------------------------- Beam -----------------------------------

Beam.Options.k_Mconsist = 1;       % Consistent Mass Matrix (CMM) = 1; 0 = LMM

% Boundary conditions variables
if all(Beam.BC.text == 'SP')
    Beam.BC.loc = [0,Beam.Prop.L]; 
    Beam.BC.vert_stiff = [-1,-1]; 
    Beam.BC.rot_stiff = [0,0];
    Beam.BC.text_long = 'Simply Supported';
elseif all(Beam.BC.text == 'FF')
    Beam.BC.loc = [0,Beam.Prop.L]; 
    Beam.BC.vert_stiff = [-1,-1]; 
    Beam.BC.rot_stiff = [-1,-1];
    Beam.BC.text_long = 'Fixed-Fixed';
end % if all(Beam.BC.text == 'SP')

% Default calculation of Beam natural frequencies
if ~isfield(Calc.Options,'calc_beam_frq')
    Calc.Options.calc_beam_frq = 1;
end % if ~isfield(Calc.Options,'calc_beam_frq')

% Default calculation of Beam modes of vibration
if ~isfield(Calc.Options,'calc_beam_modes')
    Calc.Options.calc_beam_frq = 1;
    Calc.Options.calc_beam_modes = 1;
end % if ~isfield(Calc.Options,'calc_beam_modes')

% Beam Damping
if ~isfield(Beam,'Damping')
    Beam.Damping.per = 0;
end % if ~isfield(Beam,'Damping')

% Default BM calculation method
if ~isfield(Calc.Options,'BM_calc_mode')
    Calc.Options.BM_calc_mode = 1;  % The average nodal result is considered
end % if ~isfield(Calc.Options,'BM_calc_mode')

% Default Shear calculation method
if ~isfield(Calc.Options,'Shear_calc_mode')
    Calc.Options.Shear_calc_mode = 1;  % The average nodal result is considered
end % if ~isfield(Calc.Options,'Shear_calc_mode')

% -------------------------------- Track ----------------------------------

% Rail damping
if ~isfield(Track.Rail,'Damping')
    Track.Rail.Damping.per = 0;
end % if ~isfield(Track.Rail,'Damping')

% -------------------------------- Checks ---------------------------------

if Calc.Options.num_calc_beam_sections > 0
    if any([Calc.Options.calc_beam_sections<0,Calc.Options.calc_beam_sections>Beam.Prop.L])
        disp('Beam calculation section not on beam');
        error('Beam calculation section not on beam');
    end % if any
end % if Calc.Options.num_calc_beam_sections > 0

if ~isfield(Track,'BallastOnBeam')
    Track.BallastOnBeam.included = 0;
else
    if ~isfield(Track.BallastOnBeam,'included')
        Track.BallastOnBeam.included = 1;
    end % if ~isfield(Track.BallastOnBeam,'included')
end % if ~isfield(Track,'BallastOnBeam')

if ~isfield(Track,'PadUnderSleeperOnBeam')
    Track.PadUnderSleeperOnBeam.included = 0;
else
    if ~isfield(Track.PadUnderSleeperOnBeam,'included')
        Track.PadUnderSleeperOnBeam.included = 1;
    end % if ~isfield(Track.PadUnderSleeperOnBeam,'included')
end % if ~isfield(Track,'PadUnderSleeperOnBeam')

check = Track.BallastOnBeam.included+Track.PadUnderSleeperOnBeam.included;
if check == 0
    disp('Include model properties of either:');
    disp([blanks(8),'Ballast on Beam']);
    disp([blanks(8),'Pad under Sleeper on Beam']);
    error('Missing information for "Ballast on Beam" or "Pad under Sleeper on Beam"');
elseif check == 2
    disp('Too many model properties have been defined.');
    disp('Remove the properties of one of the following:');
    disp([blanks(8),'Ballast on Beam']);
    disp([blanks(8),'Pad under Sleeper on Beam']);
    error('Properties defined simulataneously for "Ballast on Beam" and "Pad under Sleeper on Beam"');
end % if check == 0

if mod(Calc.Profile.L,Track.Sleeper.spacing) > Calc.Cte.tol
    disp('Profile length is wrong');
    error('Profile length is wrong');
end % if mod(Calc.Profile.minL,Track.Sleeper.spacing) > Calc.Cte.tol

% Redux model
if Calc.Options.redux == 0
    disp('No redux model is used! Are sure about this?');
end % if Calc.Options.redux == 0

% ---- End of function ----
