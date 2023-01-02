function [Sol] = B64_Coupled_InitialStatic(Veh,Model,Calc,Track)

% Calculation of initial static deformation of vehicle and track.
% The coupled stiffness matrix is defined and solved for the external forces

% Notes:
% The DOF of the vehicles do not correspond to the displacements of the
%   upper part of the wheels. Thus, the relation matrix N2w is needed.
% The lower part of the wheels displacements are not necessarily located on
%   top of a node of the rail. This is why the shape function is needed to
%   distribute their contributions to the according DOFs of the rail.
% The primary suspension is already included in the vehicles equations.
% But it needs to be included on the rail too.
% Also the off-diagonal block matrices need to be multiplied by the
%   stiffness of the primary suspension.

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
% Veh = Indexed structure with Veh variables, including at least:
%   ... see script ...
% Model = Structure with Model variables, including at least:
%   ... see script ...
% Calc = Structure with Calc variables, including at least:
%   ... see script ...
% Track = Structure with Track variables, including at least:
%   ... see script ...
% % ---- Outputs ----
% Sol = Additional fields to Sol structure variable
%   .Veh(i).U0 = Initial deformation of vehicles DOF
%   .Veh(i).V0 = Initial velocity of vehicles DOF
%   .Veh(i).A0 = Initial acceleration of vehicles DOF
%   .Model.Nodal.U0 = Initial deformation of coupled model
%   .Model.Nodal.V0 = Initial velocity of coupled model
%   .Model.Nodal.A0 = Initial acceleration of coupled model
% -------------------------------------------------------------------------

% ---- Initialize variables ----
Coup.DOF.Tnum = Veh(end).global_ind(end) + Model.Mesh.DOF.Tnum;
Coup.Kg = sparse(Coup.DOF.Tnum,Coup.DOF.Tnum);
%Coup.F = sparse(Coup.DOF.Tnum,1);  % Makes the solution sparse
Coup.F = zeros(Coup.DOF.Tnum,1);

% If not redux model
if Calc.Options.redux == 0

    % **** With VBI ***
    if Calc.Options.VBI == 1

    % Vehicles contributions
    for veh_num = 1:Veh(1).Tnum

        % Vehicle's Diagonal block matrices
        Coup.Kg(Veh(veh_num).global_ind,Veh(veh_num).global_ind) = ...
            Veh(veh_num).SysM.K;

        for wheel = 1:Veh(veh_num).Wheels.num

            % Element to which each wheel belongs to
            ele_num = Calc.Veh(veh_num).elexj(wheel,1);
            % Distance from each x_path to is left node
            x = Calc.Veh(veh_num).xj(wheel,1);
            % Element dimension
            a = Track.Rail.Mesh.Ele.a(ele_num);
            % Element shape functions at x
            shape_fun_at_x = Track.Rail.Mesh.shape_fun(x,a);

            % Addition primary suspension stiffness to Track
            eq_num = Veh(end).global_ind(end)+Track.Rail.Mesh.Ele.DOF(ele_num,:);
            NN = shape_fun_at_x*shape_fun_at_x';
            Coup.Kg(eq_num,eq_num) = Coup.Kg(eq_num,eq_num) + ...
                NN*Veh(veh_num).Susp.Prim.k(wheel);

            % Off-diagonal block matrices
            % Vehicle's Node to wheel displacements
            N2w = Veh(veh_num).Wheels.N2w(wheel,:);
            % Off diagonal block matrix
            OffDiagBlockMat = -(shape_fun_at_x*N2w)*Veh(veh_num).Susp.Prim.k(wheel);
            % Addition to Coupled stiffness matrix
            rows = Veh(veh_num).global_ind;
            cols = Veh(end).global_ind(end)+Track.Rail.Mesh.Ele.DOF(ele_num,:);
            Coup.Kg(rows,cols) = Coup.Kg(rows,cols) + OffDiagBlockMat';
            Coup.Kg(cols,rows) = Coup.Kg(cols,rows) + OffDiagBlockMat;

            % Force vector
            Coup.F(cols) = Veh(veh_num).Wheels.m(wheel)*shape_fun_at_x*Calc.Cte.grav;

        end % for wheel = 1:Veh(veh_num).Wheels.num

        % Force vector
        Coup.F(Veh(veh_num).global_ind) = Veh(veh_num).SysM.M * ...
            (Veh(veh_num).DOF.vert*Calc.Cte.grav);

    end % for veh_num = 1:Veh(1).Tnum

    % Track contribution
    Coup.Kg(Veh(end).global_ind(end)+1:end,Veh(end).global_ind(end)+1:end) = ...
        Coup.Kg(Veh(end).global_ind(end)+1:end,Veh(end).global_ind(end)+1:end) + ...
        Model.Mesh.Kg;

    % Re-apply the boundary conditions
    Coup.BC.DOF_fixed = Veh(end).global_ind(end) + Model.BC.DOF_fixed;
    Coup.BC.num_DOF_fixed = length(Coup.BC.DOF_fixed);
    Coup.Mg(Coup.BC.DOF_fixed,:) = 0; Coup.Mg(:,Coup.BC.DOF_fixed) = 0;
    Coup.Cg(Coup.BC.DOF_fixed,:) = 0; Coup.Cg(:,Coup.BC.DOF_fixed) = 0;
    Coup.Kg(Coup.BC.DOF_fixed,:) = 0; Coup.Kg(:,Coup.BC.DOF_fixed) = 0;
    for i = 1:Coup.BC.num_DOF_fixed
        Coup.Mg(Coup.BC.DOF_fixed(i),Coup.BC.DOF_fixed(i)) = ...
            Model.BC.DOF_fixed_value;
        Coup.Kg(Coup.BC.DOF_fixed(i),Coup.BC.DOF_fixed(i)) = ...
            Model.BC.DOF_fixed_value;
    end % for i = 1:Coup.BC.num_DOF_fixed
    Coup.F(Coup.BC.DOF_fixed) = 0;

    % **** Moving Force ****
    elseif Calc.Options.VBI == 0

        % Vehicles contributions
        for veh_num = 1:Veh(1).Tnum

            % Vehicle's Diagonal block matrices
            Coup.Kg(Veh(veh_num).global_ind,Veh(veh_num).global_ind) = ...
                Veh(veh_num).SysM.K;

            % Force vector
            Coup.F(Veh(veh_num).global_ind) = Veh(veh_num).SysM.M * ...
                (Veh(veh_num).DOF.vert*Calc.Cte.grav);

        end % for veh_num = 1:Veh(1).Tnum

        % Track contribution
        Coup.Kg(Veh(end).global_ind(end)+1:end,Veh(end).global_ind(end)+1:end) = ...
            Model.Mesh.Kg;

    end % if Calc.Options.VBI == 1

    % Coupled system static solution
    Coup.U0 = Coup.Kg\Coup.F;

elseif Calc.Options.redux == 1

    Coup.U0 = zeros(Coup.DOF.Tnum,1);

	% Vehicles initial deformations
    for veh_num = 1:Veh(1).Tnum
        Coup.U0(Veh(veh_num).global_ind) = Veh(veh_num).U0;
    end % for veh_num = 1:Veh(1).Tnum
    
end % if Calc.Options.redux == 0

% Dividing the results into Sol.Veh and Sol.Model
for veh_num = 1:Veh(1).Tnum
    Sol.Veh(veh_num).U0 = Coup.U0(Veh(veh_num).global_ind);
    Sol.Veh(veh_num).V0 = Sol.Veh(veh_num).U0*0;
    Sol.Veh(veh_num).A0 = Sol.Veh(veh_num).U0*0;
end % for veh_num = 1:Veh(1).Tnum
Sol.Model.Nodal.U0 = Coup.U0(Veh(end).global_ind(end)+1:end);
Sol.Model.Nodal.V0 = Sol.Model.Nodal.U0*0;
Sol.Model.Nodal.A0 = Sol.Model.Nodal.U0*0;

% % Graphical Check
% rows = Veh(end).global_ind(end) + Model.Mesh.DOF.rail_vert;
% figure; subplot(2,1,1); hold on; box on;
%     for veh_num = 1:Veh(1).Tnum
%         veh_wheels_u = Veh(veh_num).Wheels.N2w*Coup.U0(Veh(veh_num).global_ind);
%         plot(Calc.Veh(veh_num).x_path(:,1),veh_wheels_u*1000,'r.','MarkerSize',10);
%     end % for veh_num = 1:Veh(1).Tnum
%     xlim([Calc.Veh(end).x_path(end,1),Model.Mesh.XLoc.rail_vert(end)]);
%     xlabel('Distance (m)'); ylabel('Vehicles Vert. Disp. (mm)');
% subplot(2,1,2);
%     plot(Model.Mesh.XLoc.rail_vert,Coup.U0(rows)*1000);
%     xlim([Calc.Veh(end).x_path(end,1),Model.Mesh.XLoc.rail_vert(end)]);
%     xlabel('Distance (m)'); ylabel('Rail Vert. Disp. (mm)');

% ---- End of script ----
