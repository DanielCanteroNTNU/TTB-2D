% Core calculations of TTB-2D simulation.
% Basically this script does the following:
%   - Inputs processing and auxiliary variables generation
%   - Model generation (system matrices and load sequence)
%   - Numerical integration of solution
%   - Generation of additional results based on the solution

% Not a function.
% Uses and modifies the variables: Train, Track, Beam and Calc.
%   Generates the simulation results and saves them in variable Sol.

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

% -- Model geometry --
[Calc,Train,Beam] = B43_ModelGeometry(Calc,Train,Track,Beam);

% -- Options Processing --
[Calc,Train,Track,Beam] = B07_OptionsProcessing(Calc,Train,Track,Beam);

% ---- Beam Model ----
% Elements and coordinates
[Beam] = B01_ElementsAndCoordinates(Beam,Calc);
% Boundary conditions 
[Beam] = B02_BoundaryConditions(Beam);
% Beam system matrices (Mass and Stiffness)
[Beam] = B03_BeamMatrices(Beam);
% Beam frequencies 
[Beam] = B09_BeamFrq(Beam,Calc);
% Beam Damping Matrix
[Beam] = B24_BeamDamping(Beam);

% ---- Track Model ----
% Definition of auxiliary variables
[Track] = B51_RailVariables(Track,Calc);
% Elements and coordinates
[Track.Rail] = B01_ElementsAndCoordinates(Track.Rail);
% Boundary conditions 
[Track.Rail] = B02_BoundaryConditions(Track.Rail);
% Rail system matrices (Mass and stiffness)
[Track.Rail] = B03_BeamMatrices(Track.Rail);
% Rail frequencies
[Track.Rail] = B09_BeamFrq(Track.Rail,Calc);
% Reference frequencies for damping definition
Track.Rail.Modal.w = [zeros(Track.Rail.Modal.num_rigid_modes,1),Beam.Modal.w(1:2)];
% Beam Damping Matrix
[Track.Rail] = B24_BeamDamping(Track.Rail);

% ---- Complete Model ----
% System matrices
disp('Building model system matrices ...')
[Model] = B54_ModelMatrices(Beam,Track,Calc);
% Model boundary conditions
[Model] = B55_ModelBC(Model,Beam,Track);
% Modal analysis of Model
[Model] = B56_ModelFrq(Model,Calc);
B57_PlotModelModes(Model,Calc);
fprintf('\b'); disp(' DONE');

% ---- Vehicle Model ----
% Vehicles system matrices
[Train.Veh] = B18_TrainVehEq(Train.Veh);
% Vehicle frequencies
[Train.Veh] = B08_VehFreq(Train.Veh,Calc);
% Vehicle position and Solver time array
[Calc] = B10_EndTime(Calc);
[Calc] = B11_TimeSpaceDiscretization(Calc);
% Vehicle static loads
[Train.Veh] = B47_VehStaticLoads(Train.Veh,Calc);

% ---- Irregularity profile ----
% Generation
[Calc] = B19_GenerateProfile(Calc);
% Assigning profile to each wheel
[Calc] = B25_WheelProfiles(Calc,Train.Veh);

% -- Model visualization --
B67_PlotModelVisualization(Calc,Train.Veh,Track,Model);

% ---- Coupled Equations Solver (COUP) ----
% Element number of vertical forces in time
[Calc] = B50_ElementNumOfForce(Track.Rail,Calc);
% Initial coupled system's static deformation
[Sol] = B64_Coupled_InitialStatic(Train.Veh,Model,Calc,Track);
tic; disp('Performing Dynamic Calculations (Coupled System) ...');
[Sol] = B65_DynamicCalcCoupledFaster(Train.Veh,Model,Calc,Track,Sol);
disp(['Calculation time: ',num2str(round(toc,2)),'s']);

% ---- Additional results from solution ----
% Contact Force
[Sol] = B66_ContactForce(Sol,Track,Calc,Train);
% Beam deformation
[Sol] = B49_BeamDeformation(Sol,Model,Beam,Calc,Train,1);
% Static beam deformation
[Sol] = B49_BeamDeformation(Sol,Model,Beam,Calc,Train,0);
% Beam Bending Moment
[Sol] = B31_BeamBM(Sol,Model,Beam,Calc,1);
% Static Beam Bending Moment
[Sol] = B31_BeamBM(Sol,Model,Beam,Calc,0);
% Beam Shear Force
[Sol] = B33_BeamShear(Sol,Model,Beam,Calc,1);
% Static Beam Shear Force
[Sol] = B33_BeamShear(Sol,Model,Beam,Calc,0);
% Beam Acceleration
[Sol] = B53_BeamAcceleration(Sol,Model,Beam,Calc);
% Results at selected Beam sections
[Sol] = B58_ResultsBeamSections(Sol,Beam,Calc);

disp('All calculations finished sucessfully');

% ---- End of script ----
