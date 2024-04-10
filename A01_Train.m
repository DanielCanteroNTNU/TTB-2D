% Definitions for Train

% Not a function.
% Defines the content of the variable Train.

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

% Velocity
Train.vel = 120/3.6;          % Train velocity [m/s]

% -- Mechanical properties --

% File loading path
Train.Load.path = '';

% Defnintion of successive vehicles
veh_num = 1; run([Train.Load.path,'TrainProp_Manchester_BenchMark']);

% Optional: To add more successive vehicles to the train model, define
%   each additional vehicle/wagon as follows. Simply define the correct
%   vehicle number and call the desired vehicle properties
% Example (uncomment to use)
% veh_num = 2; run([Train.Load.path,'TrainProp_Manchester_Benchmark']);
% veh_num = 3; run([Train.Load.path,'TrainProp_Manchester_Benchmark']);

% Additional vehicle/wagon properties included in TTB-2D are:

% % AVE S103 ICE3 configuration with 8 coaches
% veh_num = 1; run([Train.Load.path,'TrainProp_AVE_S103_ICE3_Velaro_E_C1']);
% veh_num = 2; run([Train.Load.path,'TrainProp_AVE_S103_ICE3_Velaro_E_C2367']);
% veh_num = 3; run([Train.Load.path,'TrainProp_AVE_S103_ICE3_Velaro_E_C2367']);
% veh_num = 4; run([Train.Load.path,'TrainProp_AVE_S103_ICE3_Velaro_E_C45']);
% veh_num = 5; run([Train.Load.path,'TrainProp_AVE_S103_ICE3_Velaro_E_C45']);
% veh_num = 6; run([Train.Load.path,'TrainProp_AVE_S103_ICE3_Velaro_E_C2367']);
% veh_num = 7; run([Train.Load.path,'TrainProp_AVE_S103_ICE3_Velaro_E_C2367']);
% veh_num = 8; run([Train.Load.path,'TrainProp_AVE_S103_ICE3_Velaro_E_C8']);

% % Eurostar configuration
% veh_num = 1; run([Train.Load.path,'TrainProp_Eurostar_Locomotive']);
% veh_num = 2; run([Train.Load.path,'TrainProp_Eurostar_EC1']);
% veh_num = 3; run([Train.Load.path,'TrainProp_Eurostar_EC2_EC8']);
% veh_num = 4; run([Train.Load.path,'TrainProp_Eurostar_EC2_EC8']);
% veh_num = 5; run([Train.Load.path,'TrainProp_Eurostar_EC2_EC8']);
% veh_num = 6; run([Train.Load.path,'TrainProp_Eurostar_EC2_EC8']);
% veh_num = 7; run([Train.Load.path,'TrainProp_Eurostar_EC2_EC8']);
% veh_num = 8; run([Train.Load.path,'TrainProp_Eurostar_EC2_EC8']);
% veh_num = 9; run([Train.Load.path,'TrainProp_Eurostar_EC2_EC8']);
% veh_num = 10; run([Train.Load.path,'TrainProp_Eurostar_EC9']);
% veh_num = 11; run([Train.Load.path,'TrainProp_Eurostar_Locomotive']);

% % Chinese Star (Exmaple: Locomotive + 3 wagons)
% veh_num = 1; run([Train.Load.path,'TrainProp_ChineseStarPowerCar']);
% veh_num = 2; run([Train.Load.path,'TrainProp_DoubleDeckPassengerCoach']);
% veh_num = 3; run([Train.Load.path,'TrainProp_DoubleDeckPassengerCoach']);
% veh_num = 4; run([Train.Load.path,'TrainProp_DoubleDeckPassengerCoach']);

% % Pioneer M vehicle
% veh_num = 1; run([Train.Load.path,'TrainProp_Pioneer_M_vehicle']);

% % Pioneer R vehicle
% veh_num = 1; run([Train.Load.path,'TrainProp_Pioneer_R_vehicle']);

% % Shinkansen S300
% veh_num = 1; run([Train.Load.path,'TrainProp_ShinkansenS300']);

% Copy of Train.vel
Train.Veh(1).vel = Train.vel;

clear veh_num

% ---- End of script ----
