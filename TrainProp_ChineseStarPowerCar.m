% Chinese Star power car

% These vehicle properties are taken from: 
% W. Zhai, K. Wang and C. Cai, "Fundamentals of vehicle-track coupled dynamics"
% 	Vehicle System Dynamics, Vol. 47, No. 11, November 2009, 1349–1376

% Note: The vehicle model presented in A068 is in 3D. Therefore, some of the 
%   properties have to be multiplied by 2 to reduce the model to a 2D one

Train.Veh(veh_num).Body.m = 59364.2;                      % Main body mass [kg]
Train.Veh(veh_num).Body.I = 1.723e6;                      % Main body mass moment of inertia [kg*m^2]
Train.Veh(veh_num).Body.L = 5.73*2;                       % Main body length [m] (From bogie to bogie)
Train.Veh(veh_num).Body.Le = [1,1]*(1.5+3/2);             % Additional distance [m], front and back
Train.Veh(veh_num).Bogie.num = 2;                         % Number of bogies
Train.Veh(veh_num).Bogie.m = [1,1]*5630.8;                % Bogies masses [kg]
Train.Veh(veh_num).Bogie.I = [1,1]*9487;                  % Bogies mass moments of inertia [kg*m^2]
Train.Veh(veh_num).Bogie.L = [1,1]*1.5*2;                 % Bogies length [m] (From wheel to wheel)
Train.Veh(veh_num).Wheels.num = 4;                        % Number of wheels
Train.Veh(veh_num).Wheels.m = [1,1,1,1]*1843.5;           % Wheel masses [kg]
Train.Veh(veh_num).Susp.Prim.k = [1,1,1,1]*2.3996e6*2;    % Primary suspension stiffness [N/m]
Train.Veh(veh_num).Susp.Prim.c = [1,1,1,1]*30e3*2;        % Primary suspension viscous damping [Ns/m]
Train.Veh(veh_num).Susp.Sec.k = [1,1]*0.8858e6*2;         % Secondary suspension stiffness [N/m]
Train.Veh(veh_num).Susp.Sec.c = [1,1]*45e3*2;             % Secondary suspension viscous damping [Ns/m]

% ---- End of script ----