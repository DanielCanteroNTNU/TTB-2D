function [Veh] = B47_VehStaticLoads(Veh,Calc)

% Calculates the static loads of the vehicles using their system matrices 

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
% Veh = Indexed structure variable with information about each vehicle
% Calc = Structure variable with various calculation variables
% ---- Output ----
% Veh = Addition of information to Veh variable
%   .U0 = Initial deformation of each vehicle under gravity load
%   .sta_loads = Static loads produced by each wheel
% -------------------------------------------------------------------------

for veh_num = 1:Veh(1).Tnum

    % External forces (Mass * gravity)
    Fext = Veh(veh_num).SysM.M * (Veh(veh_num).DOF.vert*Calc.Cte.grav);
    % System displacements
    Veh(veh_num).U0 = Veh(veh_num).SysM.K\Fext;
    % Wheels displacements
    wheel_disp = Veh(veh_num).Wheels.N2w * Veh(veh_num).U0;
    % Static load
    Veh(veh_num).sta_loads = Veh(veh_num).Susp.Prim.k' .* ...
        wheel_disp + Veh(veh_num).Wheels.m'*Calc.Cte.grav;
    
    % Check
    check = sum(Veh(veh_num).sta_loads) - ...
        sum([Veh(veh_num).Body.m,Veh(veh_num).Bogie.m,Veh(veh_num).Wheels.m])*Calc.Cte.grav;
    if abs(check) > Calc.Cte.tol
        disp(['Static weight of vehicle ',num2str(veh_num),' is not correct']);
    end % if abs(aux1) > Calc.Cte.tol

end % for veh_num = 1:Veh(1).Tnum

% ---- End of script ----
