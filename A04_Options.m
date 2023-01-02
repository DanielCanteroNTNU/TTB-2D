% Definition of addition relevant aspects of the model and calculation options

% Not a function.
% Defines the content of the variable Calc, 
%   and modifies the content of variables Beam and Track.

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

% ------------------------- Track irregularity ----------------------------

% ---- Smooth ---- (Default)
Calc.Profile.Type = 0;

% ---- FRA ---- 
% Reference: Ladislav Fryba. Dynamics of Railway Bridges. Academia Praha, 1996
Calc.Profile.Type = 1;      % Defined from PSD [m^3]
Calc.Profile.PSD_Y_fun = @(spaf,inputs) ...
    inputs(1)*inputs(3)^2*(spaf.^(2)+inputs(2)^2)./(spaf.^(4).*(spaf.^(2)+inputs(3)^2));
%Calc.Profile.inputs(1) = 15.53*1e-8;    % A [m^3] Class 1
%Calc.Profile.inputs(1) = 8.85*1e-8;     % A [m^3] Class 2
%Calc.Profile.inputs(1) = 4.92*1e-8;     % A [m^3] Class 3
%Calc.Profile.inputs(1) = 2.75*1e-8;     % A [m^3] Class 4
%Calc.Profile.inputs(1) = 1.57*1e-8;     % A [m^3] Class 5
Calc.Profile.inputs(1) = 0.98*1e-8;     % A [m^3] Class 6
Calc.Profile.inputs(2) = 23.3*1e-3;     % Omega1 [1/m]
Calc.Profile.inputs(3) = 13.1*1e-2;     % Omega2 [1/m]
Calc.Profile.min_WaveLength = 1.524;    % [m]
Calc.Profile.max_WaveLength = 304.8;    % [m]
Calc.Profile.text = 'FRA';

% % ---- FRA version 2 ----
% % Reference: Abdur Rohim Boy Berawi. Improving railway track maintenance
% %     using power spectral density (PSD). Doctoral thesis.
% %     Link: https://sigarra.up.pt/feup/en/pub_geral.show_file?pi_doc_id=12113
% Calc.Profile.Type = 1;  % Defined from PSD [m^3]
% Calc.Profile.PSD_Y_fun = @(spaf,inputs) (inputs(1)*inputs(2)*inputs(3)^2)./((spaf.^2).*(spaf.^2+inputs(3)^2))*inputs(4);
% Calc.Profile.inputs(1) = 0.25;          % k = Some constant
% %Calc.Profile.inputs(2) = 1.2107;   % A_v [cm^2 rad/m] Class 1
% %Calc.Profile.inputs(2) = 1.0181;   % A_v [cm^2 rad/m] Class 2
% %Calc.Profile.inputs(2) = 0.6816;   % A_v [cm^2 rad/m] Class 3
% %Calc.Profile.inputs(2) = 0.5376;   % A_v [cm^2 rad/m] Class 4
% %Calc.Profile.inputs(2) = 0.2095;   % A_v [cm^2 rad/m] Class 5
% Calc.Profile.inputs(2) = 0.0339;   % A_v [cm^2 rad/m] Class 6
% Calc.Profile.inputs(3) = 0.8245;   % Omega_c [rad/m]
% Calc.Profile.inputs(4) = 1e-4/(2*pi);   % Unit conversion [cm^2 = 10^(-4) m^2] and [rad = 1/(2*pi) cycles]
% Calc.Profile.min_WaveLength = 1.524;    % [m]
% Calc.Profile.max_WaveLength = 304.8;    % [m]
% Calc.Profile.text = 'FRAv2';

% % ---- German spectra for high-speed ----
% % Reference: W.W. Guo, H. Xia, G. De Roeck, K. Liu, Integral model for 
% %     train-track-bridge interaction on the Sesia viaduct: Dynamic simulation 
% %     and critical assessment, Computers & Structures, Volumes 112–113,
% %     2012, Pages 205-216, ISSN 0045-7949,
% %     https://doi.org/10.1016/j.compstruc.2012.09.001.
% Calc.Profile.Type = 1;  % Defined from PSD [m^3]
% Calc.Profile.PSD_Y_fun = @(spaf,inputs) (inputs(1)*inputs(2)^2)./((spaf.^2+inputs(3)^2).*(spaf.^2+inputs(2)^2))*inputs(4);
% Calc.Profile.inputs(1) = 4.032e-7*1/(2*pi)*(0.4/0.9843)^2;  % A_v [m^2 rad/m]
% Calc.Profile.inputs(2) = 0.8246*1/(2*pi);                   % Omega_c [rad/m]
% Calc.Profile.inputs(3) = 0.0206*1/(2*pi);                   % Omega_T [rad/m]
% Calc.Profile.inputs(4) = 1;                         % Unit conversion [rad = 1/(2*pi) cycles]
% Calc.Profile.min_WaveLength = 3;                    % [m]
% Calc.Profile.max_WaveLength = 150;                  % [m]
% Calc.Profile.text = 'German high-speed';

% % ---- SNCF ----
% % Reference: Abdur Rohim Boy Berawi. Improving railway track maintenance
% %     using power spectral density (PSD). Doctoral thesis.
% %     Link: https://sigarra.up.pt/feup/en/pub_geral.show_file?pi_doc_id=12113
% Calc.Profile.Type = 1;  % Defined from PSD [m^3]
% Calc.Profile.PSD_Y_fun = @(spaf,inputs) inputs(1)./(1+spaf/inputs(2)).^3;
% Calc.Profile.inputs(1) = 0.509*308*10^-6;       % A for good track
% %Calc.Profile.inputs(1) = 1.790*308*10^-6;      % A for poor track
% Calc.Profile.inputs(2) = 0.0489;                % n0 [1/m]
% Calc.Profile.min_WaveLength = 2;                % [m]
% Calc.Profile.max_WaveLength = 40;               % [m]
% Calc.Profile.text = 'SNCF';

% % ---- Load existing profile ----
% Calc.Profile.Type = 5;
% Calc.Profile.Load.file_name = '20171207_FRA_Class6_500m';
% Calc.Profile.Load.path_name = 'Results\Profiles\';

% --------------------------- Mesh Properties -----------------------------

% -- Element size for Bridge model --
Beam.Mesh.Ele.num_per_spacing = 2;

% -- Element size for rail in Track model --
Track.Rail.Mesh.Ele.num_per_spacing = 2;

% -- Approach distance --
Calc.Profile.minL_Approach = 30;    % Approach distance [m]

% -- Distance after crossing -- (To allow for free vibration)
Calc.Profile.minL_After = 30;   % [m] (Default value = 10)

% ----------------- Calculation options and variables ---------------------

% Results at particular bridge sections
Calc.Options.calc_beam_sections = [Beam.Prop.L/2];      % (Default = [])

% Time step
Calc.Solver.max_accurate_frq = 500;     % [Hz]

% Consider VBI
Calc.Options.VBI = 1;        % VBI is considered (default)
%Calc.Options.VBI = 0;        % No VBI considered = Moving force solution 

% Redux model option
Calc.Options.redux = 0;         % Original script. Track under the vehicle
%Calc.Options.redux = 1;         % No track under vehicle at start and end of simulation (Default)

% Other
Calc.Options.calc_beam_modes = 0;   % In modal analysis, no modes are calculated

% -------------------------------------------------------------------------
% ------------------------- Plotting Options ------------------------------

% Note: Comment/uncomment to switch off/on the corresponding plot

% ---- Vehicle ----
%Calc.Plot.Veh.P01_VertDisp = 1;         % Vehicle vertical diplacements
%Calc.Plot.Veh.P02_VertVel = 1;          % Vehicle vertical velocities
Calc.Plot.Veh.P03_VertAcc = 1;          % Vehicle vertical accelerations
Calc.Plot.Veh.P04_ContactForce_t = 1;   % Vehicle vertical contact force in time (1 = Values; 2 = With 0 limit)
%Calc.Plot.Veh.P05_ContactForce_x = 1;   % Vehicle vertical contact force in space (1 = Values; 2 = With 0 limit)

% ---- Model ----
Calc.Plot.Model.P00_ModelVisualization = 1;% Sketch of model before performing simulation
%Calc.Plot.Model.P01_ModelDef = 50;       % Model vertical deformation at X% of the total simulated time
%Calc.Plot.Model.P02_ModelRot = 10;       % Model rotation values at X% of the total simulated time

% ---- Beam ----
%Calc.Plot.Beam.P01_DispContour = 1;          % Beam displacement contour plot
Calc.Plot.Beam.P02_StaticDispContour = 1;    % Beam static displacment contour plot
Calc.Plot.Beam.P03_BMContour = 1;            % Beam Bending moment contour plot
%Calc.Plot.Beam.P04_StaticBMContour = 1;      % Beam Static Bending moment contour plot
%Calc.Plot.Beam.P05_ShearContour = 1;         % Beam Shear contour plot
%Calc.Plot.Beam.P06_StaticShearContour = 1;   % Beam Static Shear contour plot
%Calc.Plot.Beam.P07_VertAccContour = 1;       % Beam Vertial Acceleration contour plot
%Calc.Plot.Beam.P08_Sections_BeamVertDisp = 1;% Beam vertical displacement at selected sections
%Calc.Plot.Beam.P09_Sections_BeamBM = 1;      % Beam Bending Moment at selected sections
%Calc.Plot.Beam.P10_Sections_BeamShear = 1;   % Beam Shear at selected sections
Calc.Plot.Beam.P11_Sections_BeamAcc = 1;     % Beam Acceleration at selected sections

% ---- Others ----
%Calc.Plot.Profile_original = 1;         % Generated Profile
%Calc.Plot.P3_VehPos = 1;                % Vehicle position in time

% ---- End of script ----
