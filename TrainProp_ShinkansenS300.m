% Shinkansen S300

% These vehicle properties are taken from:
% Yean-Seng Wu, Yeong-Bin Yang, Steady-state response and riding comfort of
%   trains moving over a series of simply supported bridges, Engineering 
%   Structures, Volume 25, Issue 2, 2003, Pages 251-265, ISSN 0141-0296,
%   https://doi.org/10.1016/S0141-0296(02)00147-5.

Train.Veh(veh_num).Body.m = 41750;                        % Main body mass [kg]
Train.Veh(veh_num).Body.I = 2080000;                      % Main body mass moment of inertia [kg*m^2]
Train.Veh(veh_num).Body.L = 17.5;                         % Main body length [m] (From bogie to bogie)
Train.Veh(veh_num).Body.Le = [1,1]*(25-17.5)/2;           % Additional distance [m], front and back
Train.Veh(veh_num).Bogie.num = 2;                         % Number of bogies
Train.Veh(veh_num).Bogie.m = [1,1]*3040;                  % Bogies masses [kg]
Train.Veh(veh_num).Bogie.I = [1,1]*3930;                  % Bogies mass moments of inertia [kg*m^2]
Train.Veh(veh_num).Bogie.L = [1,1]*2.5;                   % Bogies length [m] (From wheel to wheel)
Train.Veh(veh_num).Wheels.num = 4;                        % Number of wheels
Train.Veh(veh_num).Wheels.m = [1,1,1,1]*1780;             % Wheel masses [kg]
Train.Veh(veh_num).Susp.Prim.k = [1,1,1,1]*1180e3;        % Primary suspension stiffness [N/m]
Train.Veh(veh_num).Susp.Prim.c = [1,1,1,1]*39.2e3;        % Primary suspension viscous damping [Ns/m]
Train.Veh(veh_num).Susp.Sec.k = [1,1]*530e3;              % Secondary suspension stiffness [N/m]
Train.Veh(veh_num).Susp.Sec.c = [1,1]*90.2e3;             % Secondary suspension viscous damping [Ns/m]

% ---- End of script ----