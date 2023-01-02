function [Calc] = B50_ElementNumOfForce(Beam,Calc)

% Calculates the element and relative position of each wheel for each vehicle in time

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
% % ---- Inputs ----
% Beam = Structure with Beam's variables, including at least:
%   ... see script ...
% Calc = Structure with Calculation variables, including at least:
%   ... see script ...
% % ---- Outputs ----
% Calc = Additional fields to Calc structure variable
%   ... see script ...
% -------------------------------------------------------------------------

% Vehicle loop
for veh_num = 1:size(Calc.Veh,2)

    % Determination of element number
    for wheel = 1:size(Calc.Veh(veh_num).x_path,1)
        aux1 = Calc.Veh(veh_num).x_path(wheel,:);
        elexj = aux1*0; xj = elexj;
        for j = Beam.Mesh.Ele.Tnum:-1:1
            aux4 = aux1>=Beam.Mesh.Nodes.acum(j);
            elexj(aux4) = j;
            xj(aux4) = aux1(aux4) - Beam.Mesh.Nodes.acum(elexj(aux4));
            aux1(aux4) = -1;
        end % for j = Beam.Mesh.Ele.Tnum:-1:1
        Calc.Veh(veh_num).elexj(wheel,:) = elexj;
        Calc.Veh(veh_num).xj(wheel,:) = xj;
    end % for wheel = 1:size(Calc.Veh(veh_num).x_path,1)

    if Calc.Options.redux == 1
        ind = or(Calc.Veh(veh_num).x_path<0,Calc.Veh(veh_num).x_path>Calc.Profile.L);
        Calc.Veh(veh_num).elexj(ind) = 0;
        Calc.Veh(veh_num).xj(ind) = 0;
    end % if Calc.Options.redux == 1
    
%     % Graphical check
%     figure; subplot(2,1,1); 
%         plot(Calc.Solver.t,Calc.Veh(veh_num).elexj');
%         ylabel('Element num.');
%     subplot(2,1,2); 
%         plot(Calc.Veh(veh_num).xj');
%         ylabel('x in element');

end % for veh_num = 1:Veh(1).Tnum

% ---- End of function ----
