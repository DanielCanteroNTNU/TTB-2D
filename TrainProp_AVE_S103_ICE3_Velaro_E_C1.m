% AVE S103 - Coach 1

% These vehicle properties are taken from: 
% Jose Goicolea (2014) Simplified Mechanical Description of AVE S-103 -ICE3
%   Velaro E High Speed Train. Available at: https://oa.upm.es/43946/1/aves103_ice3.pdf

% Note: The vehicle model properties in this document correspond to a 3D model.
%	Therefore, some of the properties have to be multiplied by 2 to reduce the model to 2D

Train.Veh(veh_num).Body.m = 47800;                        % Main body mass [kg]
Train.Veh(veh_num).Body.I = 1957888;                      % Main body mass moment of inertia [kg*m^2]
Train.Veh(veh_num).Body.L = 17.375;                       % Main body length [m] (From bogie to bogie)
Train.Veh(veh_num).Body.Le = [4.76,7.4/2];                % Additional distance [m], front and back
Train.Veh(veh_num).Bogie.num = 2;                         % Number of bogies
Train.Veh(veh_num).Bogie.m = [1,1]*3500;                  % Bogies masses [kg]
Train.Veh(veh_num).Bogie.I = [1,1]*1715;                  % Bogies mass moments of inertia [kg*m^2]
Train.Veh(veh_num).Bogie.L = [1,1]*2.5;                   % Bogies length [m] (From wheel to wheel)
Train.Veh(veh_num).Wheels.num = 4;                        % Number of wheels
Train.Veh(veh_num).Wheels.m = [1,1,1,1]*1800;             % Wheel masses [kg]
Train.Veh(veh_num).Susp.Prim.k = [1,1,1,1]*1200e3*2;      % Primary suspension stiffness [N/m]
Train.Veh(veh_num).Susp.Prim.c = [1,1,1,1]*10e3*2;        % Primary suspension viscous damping [Ns/m]
Train.Veh(veh_num).Susp.Sec.k = [1,1]*350e3*2;            % Secondary suspension stiffness [N/m]
Train.Veh(veh_num).Susp.Sec.c = [1,1]*20e3*2;             % Secondary suspension viscous damping [Ns/m]

% ---- End of script ----