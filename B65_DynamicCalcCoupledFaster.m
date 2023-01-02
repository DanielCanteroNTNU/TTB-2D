function [Sol] = B65_DynamicCalcCoupledFaster(Veh,Model,Calc,Track,Sol)

% Faster alternative version of script B65. The main difference is that the
% system matrices are assembled using the sparse() command

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
% Sol = Structure with Sol variables, including at least:
%   ... see script ...
% % ---- Outputs ----
% Sol = Additional fields to Sol structure variable
%   .Veh(i).U = Deformation in time of vehicles DOF
%   .Veh(i).V = Velocity in time of vehicles DOF
%   .Veh(i).A = Acceleration in time of vehicles DOF
%   .Model.Nodal.U = Deformation in time of coupled model
%   .Model.Nodal.V = Velocity in time of coupled model
%   .Model.Nodal.A = Acceleration in time of coupled model
% -------------------------------------------------------------------------

% ---- Initialize variables ----
Coup.DOF.Tnum = Veh(end).global_ind(end) + Model.Mesh.DOF.Tnum;
UnCoup.Kg = sparse(Coup.DOF.Tnum,Coup.DOF.Tnum);
UnCoup.Cg = UnCoup.Kg;
UnCoup.Mg = UnCoup.Kg;
UnCoup.F = sparse(Coup.DOF.Tnum,1);
Coup.U = zeros(Coup.DOF.Tnum,Calc.Solver.num_t);
Coup.V = Coup.U;
Coup.A = Coup.U;

% ---- Coupled equations BC ----
Coup.BC.DOF_fixed = Veh(end).global_ind(end) + Model.BC.DOF_fixed;
Coup.BC.num_DOF_fixed = length(Coup.BC.DOF_fixed);

% ---- Auxiliary variables ----
Aux.disp_every_t = Calc.Options.disp_every;
Aux.PCtime_start = clock;
Aux.last_display_time = Aux.disp_every_t;
vel = Veh(1).vel;
vel2 = vel^2;
ele_DOF = Track.Rail.Mesh.Ele.DOF;
global_ind_end = Veh(end).global_ind(end);
grav = Calc.Cte.grav;
NB_cte(1) = 1/(Calc.Solver.NewMark_beta*Calc.Solver.dt^2);
NB_cte(2) = Calc.Solver.NewMark_delta/(Calc.Solver.NewMark_beta*Calc.Solver.dt);
NB_cte(3) = 1/(Calc.Solver.NewMark_beta*Calc.Solver.dt);
NB_cte(4) = (1/(2*Calc.Solver.NewMark_beta)-1);
NB_cte(5) = (1-Calc.Solver.NewMark_delta/Calc.Solver.NewMark_beta);
NB_cte(6) = (1-Calc.Solver.NewMark_delta/(2*Calc.Solver.NewMark_beta))*Calc.Solver.dt;

% ---- Initial deformation ----
for veh_num = 1:Veh(1).Tnum
    Coup.U(Veh(veh_num).global_ind,1) = Sol.Veh(veh_num).U0;
    Coup.V(Veh(veh_num).global_ind,1) = Sol.Veh(veh_num).V0;
    Coup.A(Veh(veh_num).global_ind,1) = Sol.Veh(veh_num).A0;
end % for veh_num = 1:Veh(1).Tnum
Coup.U(Veh(veh_num).global_ind(end)+1:end,1) = Sol.Model.Nodal.U0;
Coup.V(Veh(veh_num).global_ind(end)+1:end,1) = Sol.Model.Nodal.V0;
Coup.A(Veh(veh_num).global_ind(end)+1:end,1) = Sol.Model.Nodal.A0;

% ---- Uncoupled System Matrices ----
% These are the system matrices of vehicles + track that are independent of 
%   the vehicle's position.

% Vehicles contributions
for veh_num = 1:Veh(1).Tnum
    
    % Vehicle's Diagonal block matrices
    UnCoup.Kg(Veh(veh_num).global_ind,Veh(veh_num).global_ind) = ...
        Veh(veh_num).SysM.K;
    UnCoup.Cg(Veh(veh_num).global_ind,Veh(veh_num).global_ind) = ...
        Veh(veh_num).SysM.C;
    UnCoup.Mg(Veh(veh_num).global_ind,Veh(veh_num).global_ind) = ...
        Veh(veh_num).SysM.M;

    % Force vector
    UnCoup.F(Veh(veh_num).global_ind) = ...
        Veh(veh_num).SysM.M * (Veh(veh_num).DOF.vert*grav);
    
end % for veh_num = 1:Veh(1).Tnum

% Track contribution
UnCoup.Kg(global_ind_end+1:end,global_ind_end+1:end) = Model.Mesh.Kg;
UnCoup.Cg(global_ind_end+1:end,global_ind_end+1:end) = Model.Mesh.Cg;
UnCoup.Mg(global_ind_end+1:end,global_ind_end+1:end) = Model.Mesh.Mg;

% % Symmetry check
% [sum(sum(abs(UnCoup.Kg-UnCoup.Kg')))==0, ...
%     sum(sum(abs(UnCoup.Cg-UnCoup.Cg')))==0, ...
%     sum(sum(abs(UnCoup.Mg-UnCoup.Mg')))==0]

% **** With VBI ***
if Calc.Options.VBI == 1

% --------------------------- Time Step Loop ------------------------------
% Time Step Loop
for t = 1:Calc.Solver.num_t-1

    % Progress display
    Aux = F01_Progress_Display(Aux,t,Calc.Solver.num_t);
    
    % ---- Time dependant system matrices ----
    Coup.Kg = UnCoup.Kg;
    Coup.Cg = UnCoup.Cg;
    Coup.Mg = UnCoup.Mg;
    Coup.F = UnCoup.F;

    vals_Kg = [];
    vals_Cg = [];
    vals_Mg = [];
    vals_Kg_off = [];
    vals_Cg_off = [];
    vals_F = [];
    eq_num1 = [];
    eq_num2 = [];
    eq_num1_off = [];
    eq_num2_off = [];
    eq_numF = [];

    % Vehicles contributions
    for veh_num = 1:Veh(1).Tnum

        ks = Veh(veh_num).Susp.Prim.k;
        cs = Veh(veh_num).Susp.Prim.c;
        ms = Veh(veh_num).Wheels.m;
        ele_num_t_1 = Calc.Veh(veh_num).elexj(:,t+1);
        N2w_wheels = Veh(veh_num).Wheels.N2w;
        h_path_t_1 = Calc.Veh(veh_num).h_path(:,t+1);
        hd_path_t_1 = Calc.Veh(veh_num).hd_path(:,t+1);
        hdd_path_t_1 = Calc.Veh(veh_num).hdd_path(:,t+1);
        rows = Veh(veh_num).global_ind;     % Modification 2015-01-13

        for wheel = 1:Veh(veh_num).Wheels.num

            % Element to which each wheel belongs to
            ele_num = ele_num_t_1(wheel);

            if ele_num > 0

                % Distance from each x_path to is left node
                x = Calc.Veh(veh_num).xj(wheel,t+1);

                % Element dimension
                a = Track.Rail.Mesh.Ele.a(ele_num);

                % Element shape functions at x
                shape_fun_at_x = Track.Rail.Mesh.shape_fun(x,a);
                shape_fun_at_x_p = Track.Rail.Mesh.shape_fun_p(x,a);
                shape_fun_at_x_pp = Track.Rail.Mesh.shape_fun_pp(x,a);

                % Auxiliary matrices
                NN = shape_fun_at_x*shape_fun_at_x';
                NNp = shape_fun_at_x*shape_fun_at_x_p';
                NNpp = shape_fun_at_x*shape_fun_at_x_pp';

                % Addition primary suspension porperties to Track
                cols = global_ind_end+ele_DOF(ele_num,:);
                eq_num_aux1 = repmat(cols',4,1);
                eq_num1 = [eq_num1;eq_num_aux1];
                eq_num_aux2 = repmat(cols,4,1);
                eq_num2 = [eq_num2;eq_num_aux2(:)];
                vals_Kg_aux = NN*ks(wheel) + cs(wheel)*vel*NNp + ms(wheel)*vel2*NNpp;
                vals_Cg_aux = NN*cs(wheel) + 2*ms(wheel)*vel*NNp;

                % Addition wheel masses to Track
                vals_Mg_aux = NN*ms(wheel);

                vals_Kg = [vals_Kg; vals_Kg_aux(:)];
                vals_Cg = [vals_Cg; vals_Cg_aux(:)];
                vals_Mg = [vals_Mg; vals_Mg_aux(:)];

                % Off-diagonal block matrices

                % Vehicle's Node to wheel displacements
                N2w = N2w_wheels(wheel,:);

                % Off-diagonal block matrix
                OffDiagBlockMat = -(shape_fun_at_x*N2w);
                OffDiagBlockMat_d = -(shape_fun_at_x_p*N2w)*vel;

                % Addition to Coupled stiffness matrix
                eq_num_aux1 = repmat(rows',4,1);
                eq_num_aux2 = repmat(cols,Veh(veh_num).Tnum_DOF,1);
                eq_num1_off = [eq_num1_off;eq_num_aux1;eq_num_aux2(:)];
                eq_num2_off = [eq_num2_off;eq_num_aux2(:);eq_num_aux1];
                vals_Kg_aux = (OffDiagBlockMat*ks(wheel) + OffDiagBlockMat_d*cs(wheel))';
                vals_Kg_aux2 = (OffDiagBlockMat*ks(wheel))';

                % Addition to Coupled damping matrix
                vals_Cg_aux = (OffDiagBlockMat*cs(wheel))';
                vals_Kg_off = [vals_Kg_off; vals_Kg_aux(:); vals_Kg_aux2(:)];
                vals_Cg_off = [vals_Cg_off; vals_Cg_aux(:); vals_Cg_aux(:)];

                % Force vector
                vals_F = [vals_F; (ms(wheel)*grav + ...
                    - ms(wheel)*hdd_path_t_1(wheel))*shape_fun_at_x; ...
                    (ks(wheel)*h_path_t_1(wheel) + cs(wheel)*hd_path_t_1(wheel))*[N2w';-shape_fun_at_x]];
                eq_numF = [eq_numF,cols,rows,cols];

            end % if ele_num > 0
        end % for wheel = 1:Veh(veh_num).Wheels.num
    end % for veh_num = 1:Veh(1).Tnum

    % Adding the coupling terms as sparse elements according to their position in the whole matrix
    Coup.Kg = Coup.Kg + sparse(eq_num1,eq_num2,vals_Kg,Coup.DOF.Tnum,Coup.DOF.Tnum) + ...
        sparse(eq_num1_off,eq_num2_off,vals_Kg_off,Coup.DOF.Tnum,Coup.DOF.Tnum);
    Coup.Cg = Coup.Cg + sparse(eq_num1,eq_num2,vals_Cg,Coup.DOF.Tnum,Coup.DOF.Tnum) + ...
        sparse(eq_num1_off,eq_num2_off,vals_Cg_off,Coup.DOF.Tnum,Coup.DOF.Tnum);
    Coup.Mg = Coup.Mg + sparse(eq_num1,eq_num2,vals_Mg,Coup.DOF.Tnum,Coup.DOF.Tnum);
    Coup.F = Coup.F + sparse(eq_numF,1,vals_F,Coup.DOF.Tnum,1);

    % Re-apply the boundary conditions
    Coup.Mg(Coup.BC.DOF_fixed,:) = 0; Coup.Mg(:,Coup.BC.DOF_fixed) = 0;
    Coup.Cg(Coup.BC.DOF_fixed,:) = 0; Coup.Cg(:,Coup.BC.DOF_fixed) = 0;
    Coup.Kg(Coup.BC.DOF_fixed,:) = 0; Coup.Kg(:,Coup.BC.DOF_fixed) = 0;
    for i = 1:Coup.BC.num_DOF_fixed
        Coup.Mg(Coup.BC.DOF_fixed(i),Coup.BC.DOF_fixed(i)) = Model.BC.DOF_fixed_value;
        Coup.Kg(Coup.BC.DOF_fixed(i),Coup.BC.DOF_fixed(i)) = Model.BC.DOF_fixed_value;
    end % for i = 1:Coup.BC.num_DOF_fixed
    Coup.F(Coup.BC.DOF_fixed) = 0;

    % ---- Direct integraion ----

    % -- Newmark-Beta --
    % Effective Stiffness Matrix
    effKg = Coup.Kg + NB_cte(1)*Coup.Mg + NB_cte(2)*Coup.Cg;
    % Newmark-beta scheme
    A = Coup.U(:,t)*NB_cte(1) + Coup.V(:,t)*NB_cte(3) + Coup.A(:,t)*NB_cte(4);
    B = (NB_cte(2)*Coup.U(:,t) - NB_cte(5)*Coup.V(:,t) - NB_cte(6)*Coup.A(:,t));
    Coup.U(:,t+1) = effKg\(Coup.F + Coup.Mg*A + Coup.Cg*B);
    Coup.V(:,t+1) = NB_cte(2)*Coup.U(:,t+1) - B;
    Coup.A(:,t+1) = Coup.U(:,t+1)*NB_cte(1) - A;

end % for t = 1:Calc.Solver.num_t-1

% **** Moving Forces ***
elseif Calc.Options.VBI == 0

    disp('No VBI!!!');
    
    % ---- System matrices ----
    Coup.Kg = UnCoup.Kg;
    Coup.Cg = UnCoup.Cg;
    Coup.Mg = UnCoup.Mg;

    % -- Effective Stiffness Matrix --
    effKg = Coup.Kg + Coup.Mg/(Calc.Solver.NewMark_beta*Calc.Solver.dt^2) + ...
        Calc.Solver.NewMark_delta/(Calc.Solver.NewMark_beta*Calc.Solver.dt)*Coup.Cg;

    % Time Step Loop
    for t = 1:Calc.Solver.num_t-1

        % Progress display
        Aux = F01_Progress_Display(Aux,t,Calc.Solver.num_t);    

        % ---- Time dependant system matrices ----
        Coup.F = UnCoup.F;

        % Vehicles contributions
        for veh_num = 1:Veh(1).Tnum

            % -- Original script --
            for wheel = 1:Veh(veh_num).Wheels.num
                % Element to which each wheel belongs to
                ele_num = Calc.Veh(veh_num).elexj(wheel,t+1);
                if ele_num > 0
                    % Distance from each x_path to is left node
                    x = Calc.Veh(veh_num).xj(wheel,t+1);
                    % Element dimension
                    a = Track.Rail.Mesh.Ele.a(ele_num);
                    % Element shape functions at x
                    shape_fun_at_x = Track.Rail.Mesh.shape_fun(x,a);
                    % Force vector components
                    cols = Veh(end).global_ind(end)+Track.Rail.Mesh.Ele.DOF(ele_num,:);
                    Coup.F(cols) = Veh(veh_num).sta_loads(wheel)*shape_fun_at_x;
                end % if ele_num > 0
            end % for wheel = 1:Veh(veh_num).Wheels.num

        end % for veh_num = 1:Veh(1).Tnum

        % Re-apply the boundary conditions
        Coup.F(Coup.BC.DOF_fixed) = 0;

        % ---- Direct integraion ----
        % Newmark-beta scheme
        A = Coup.U(:,t)/(Calc.Solver.NewMark_beta*Calc.Solver.dt^2) + ...
                Coup.V(:,t)/(Calc.Solver.NewMark_beta*Calc.Solver.dt) + ...
                Coup.A(:,t)*(1/(2*Calc.Solver.NewMark_beta)-1);
        B = (Calc.Solver.NewMark_delta/(Calc.Solver.NewMark_beta*Calc.Solver.dt)*Coup.U(:,t) - ...
                (1-Calc.Solver.NewMark_delta/Calc.Solver.NewMark_beta)*Coup.V(:,t) - ...
                (1-Calc.Solver.NewMark_delta/(2*Calc.Solver.NewMark_beta))*Calc.Solver.dt*Coup.A(:,t));
        Coup.U(:,t+1) = effKg\(Coup.F + Coup.Mg*A + Coup.Cg*B);
        Coup.V(:,t+1) = Calc.Solver.NewMark_delta/(Calc.Solver.NewMark_beta*Calc.Solver.dt)*Coup.U(:,t+1) - B;
        Coup.A(:,t+1) = Coup.U(:,t+1)/(Calc.Solver.NewMark_beta*Calc.Solver.dt^2) - A;

    end % for t = 1:Calc.Solver.num_t-1
end % if Calc.Options.VBI == 1

% ---- Output generation ----
% Dividing the results into Sol.Veh and Sol.Model
for veh_num = 1:Veh(1).Tnum
    Sol.Veh(veh_num).U = Coup.U(Veh(veh_num).global_ind,:);
    Sol.Veh(veh_num).V = Coup.V(Veh(veh_num).global_ind,:);
    Sol.Veh(veh_num).A = Coup.A(Veh(veh_num).global_ind,:);
end % for veh_num = 1:Veh(1).Tnum
Sol.Model.Nodal.U = Coup.U(global_ind_end+1:end,:);
Sol.Model.Nodal.V = Coup.V(global_ind_end+1:end,:);
Sol.Model.Nodal.A = Coup.A(global_ind_end+1:end,:);

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

function [Aux] = F01_Progress_Display(Aux,t,num_t)

% Display the progress of the calculation

% -- Inputs --
% Aux = Auxiliary structure with fields:
%   .disp_every_t = Display information every X seconds
%   .PCtime_start = Time of start of calculation
%   .last_display_time = When was the information displayed last time

% -- Output --
% Aux = The same structure but with updated values

Aux.PCtime = etime(clock,Aux.PCtime_start);

if Aux.PCtime > Aux.last_display_time
    disp_text = ['Time step ',num2str(t-1),' of ',num2str(num_t),...
        ' (',num2str(round((t-1)/num_t*100,2)),'%)'];
    disp(disp_text);
    Aux.last_display_time = Aux.last_display_time + Aux.disp_every_t;
end % if Aux.PCtime > Aux.last_display_time

end % function [Aux] = F01_Progress_Display(Aux,t,num_t)

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

end % function [Sol] = B65_DynamicCalcCoupledFaster(Veh,Model,Calc,Track,Sol)

% ---- End of script ----
