function [Fextnew] = B14_EqVertNodalForce(Beam,Calc)

% Distribute the forces to the degrees of freedom

% For each time step and depending on the location of the force, the actual
% forces must be distributed to the degrees of freedom, this can be
% accomplished through the shape functions.
% NOTE: Only Vertical forces calculated

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
% Fextnew = Equivalent nodal forces vector
% -------------------------------------------------------------------------

% -------------------------- Original script ------------------------------
% Note: This is the original script, which is easier to read. However, it
%   is more efficient to generate the force matrix using the sparse command
% 
% % Initialize variable
% Fextnew = zeros(Beam.Mesh.DOF.Tnum,Calc.Solver.num_t);
% 
% % Vehicle loop
% for veh_num = 1:size(Calc.Veh,2)
% 
%     % Number of loads loop
%     for wheel = 1:size(Calc.Veh(veh_num).x_path,1)
% 
%         %Fextnew1 = Fextnew*0;  % Slightly slower
%         Fextnew1 = zeros(Beam.Mesh.DOF.Tnum,Calc.Solver.num_t);
%         
% 	% ---- Time loop alternative ----
%         for t = 1:Calc.Solver.num_t
% %            if Calc.Veh(veh_num).elexj(wheel,t) > 0
%                 
%                 elex = Calc.Veh(veh_num).elexj(wheel,t); 
%                 x = Calc.Veh(veh_num).xj(wheel,t);
%                 a = Beam.Mesh.Ele.a(elex);
% 
%                 % DOFs for the element    
%                 ele_DOF = Beam.Mesh.Ele.DOF(elex,:);
% 
%                 % Multiplication of nodal displacements by corresponding shape function value
%                 Fextnew1(ele_DOF,t) = Calc.Veh(veh_num).F_onBeam(wheel,t)*Beam.Mesh.shape_fun(x,a);
%                 
% %            end % if Calc.Veh(veh_num).elexj(wheel,t) > 0
%         end % for t = 1:Calc.Solver.num_t
% 
%     Fextnew = Fextnew + Fextnew1;
% 
%     end % for wheel = 1:length(Ry)
% end % for veh_num = 1:Veh(1).Tnum
% 
% % Application of boundary conditoins to force vector
% %Fextnew(Beam.bc,:) = 0;  % Vertical force = 0
% Fextnew(Beam.BC.DOF_fixed,:) = 0;  % Vertical force = 0

% -------------------------- Sparse Alternative ---------------------------

% Initialize variables
rows  = [];
cols  = [];
F_vals = [];
ones_1_4 = ones(1,4);

% Vehicle loop
for veh_num = 1:size(Calc.Veh,2)

    % Number of loads loop
    for wheel = 1:size(Calc.Veh(veh_num).x_path,1)

        % Time loop
        for t = 1:Calc.Solver.num_t

            elex = Calc.Veh(veh_num).elexj(wheel,t); 
            
            if elex > 0
                
                x = Calc.Veh(veh_num).xj(wheel,t);
                a = Beam.Mesh.Ele.a(elex);

                % DOFs for the element    
                ele_DOF = Beam.Mesh.Ele.DOF(elex,:);

                rows = [rows, ele_DOF];
                cols = [cols, t*ones_1_4];
                F_vals = [F_vals; Calc.Veh(veh_num).F_onBeam(wheel,t)*Beam.Mesh.shape_fun(x,a)];
                
            end % if elex > 0
        end % for t = 1:Calc.Solver.num_t
    end % for wheel = 1:length(Ry)
end % for veh_num = 1:Veh(1).Tnum

Fextnew = sparse(rows,cols,F_vals,Beam.Mesh.DOF.Tnum,Calc.Solver.num_t);

% Application of boundary conditoins to force vector
Fextnew(Beam.BC.DOF_fixed,:) = 0;  % Vertical force = 0

% ---- End of function ----
