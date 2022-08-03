% The Manchester Benchmark vehicle

% These vehicle properties are taken from:
% Simon Iwnick (1998) Manchester Benchmarks for Rail Vehicle Simulation, 
%   Vehicle System Dynamics: International Journal of Vehicle Mechanics and 
%   Mobility, 30:3-4, 295-313, DOI: 10.1080/00423119808969454

% Note: The vehicle model presented in the source document is in 3D. Therefore,
%   some of the properties have to be multiplied by 2 to reduce the model to 2D

Train.Veh(veh_num).Body.m = 32000;                        % Main body mass [kg]
Train.Veh(veh_num).Body.I = 1970000;                      % Main body mass moment of inertia [kg*m^2]
Train.Veh(veh_num).Body.L = 9.5*2;                        % Main body length [m] (From bogie to bogie)
Train.Veh(veh_num).Body.Le = [1,1]*(1.5+3/2);             % Additional distance [m], front and back
Train.Veh(veh_num).Bogie.num = 2;                         % Number of bogies
Train.Veh(veh_num).Bogie.m = [1,1]*2615;                  % Bogies masses [kg]
Train.Veh(veh_num).Bogie.I = [1,1]*1476;                  % Bogies mass moments of inertia [kg*m^2]
Train.Veh(veh_num).Bogie.L = [1,1]*1.28*2;                % Bogies length [m] (From wheel to wheel)
Train.Veh(veh_num).Wheels.num = 4;                        % Number of wheels
Train.Veh(veh_num).Wheels.m = [1,1,1,1]*1813;             % Wheel masses [kg]
Train.Veh(veh_num).Susp.Prim.k = [1,1,1,1]*1200e3*2;      % Primary suspension stiffness [N/m]
Train.Veh(veh_num).Susp.Prim.c = [1,1,1,1]*4e3*2;         % Primary suspension viscous damping [Ns/m]
Train.Veh(veh_num).Susp.Sec.k = [1,1]*430e3*2;            % Secondary suspension stiffness [N/m]
Train.Veh(veh_num).Susp.Sec.c = [1,1]*20e3*2;             % Secondary suspension viscous damping [Ns/m]

% ---- End of script ----