% Pioneer R vehicle 

% These vehicle properties are taken from:
% Pablo Antolín, Nan Zhang, José M. Goicolea, He Xia, Miguel Á. Astiz, 
%   Javier Oliva, Consideration of nonlinear wheel–rail contact forces for 
%   dynamic vehicle–bridge interaction in high-speed railways, Journal of 
%   Sound and Vibration, Volume 332, Issue 5, 2013, Pages 1231-1251,
%   ISSN 0022-460X, https://doi.org/10.1016/j.jsv.2012.10.022.

Train.Veh(veh_num).Body.m = 44000;                        % Main body mass [kg]
Train.Veh(veh_num).Body.I = 2740000;                      % Main body mass moment of inertia [kg*m^2]
Train.Veh(veh_num).Body.L = 18;                           % Main body length [m] (From bogie to bogie)
Train.Veh(veh_num).Body.Le = [1,1]*(18-0)/2;              % Additional distance [m], front and back
Train.Veh(veh_num).Bogie.num = 2;                         % Number of bogies
Train.Veh(veh_num).Bogie.m = [1,1]*1700;                  % Bogies masses [kg]
Train.Veh(veh_num).Bogie.I = [1,1]*1700;                  % Bogies mass moments of inertia [kg*m^2]
Train.Veh(veh_num).Bogie.L = [1,1]*2.5;                   % Bogies length [m] (From wheel to wheel)
Train.Veh(veh_num).Wheels.num = 4;                        % Number of wheels
Train.Veh(veh_num).Wheels.m = [1,1,1,1]*1900;             % Wheel masses [kg]
Train.Veh(veh_num).Susp.Prim.k = [1,1,1,1]*700e3;        % Primary suspension stiffness [N/m]
Train.Veh(veh_num).Susp.Prim.c = [1,1,1,1]*38e3;          % Primary suspension viscous damping [Ns/m]
Train.Veh(veh_num).Susp.Sec.k = [1,1]*350e3;              % Secondary suspension stiffness [N/m]
Train.Veh(veh_num).Susp.Sec.c = [1,1]*40e3;               % Secondary suspension viscous damping [Ns/m]

% ---- End of script ----