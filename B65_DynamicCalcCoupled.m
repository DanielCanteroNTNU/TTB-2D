function [Sol] = B65_DynamicCalcCoupled(Veh,Model,Calc,Track,Sol)

% Note: There is an alternative version! This one is not optimized.
%	However, it is left here for reference on how system matrices are
%   coupled together

% Direct numerical integration of the coupled problem. The system matrices
%   are assembled for each integration step, and solved numerically with the
%   Newmark-Beta algorithm

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
%UnCoup.Kg = zeros(Coup.DOF.Tnum,Coup.DOF.Tnum); % Very slow option
UnCoup.Cg = UnCoup.Kg;
UnCoup.Mg = UnCoup.Kg;
UnCoup.F = sparse(Coup.DOF.Tnum,1);
%UnCoup.F = zeros(Coup.DOF.Tnum,1); % Very slow option
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
    UnCoup.Kg(Veh(veh_num).global_ind,Veh(veh_num).global_ind) = Veh(veh_num).SysM.K;
    UnCoup.Cg(Veh(veh_num).global_ind,Veh(veh_num).global_ind) = Veh(veh_num).SysM.C;
    UnCoup.Mg(Veh(veh_num).global_ind,Veh(veh_num).global_ind) = Veh(veh_num).SysM.M;

    % Force vector
    UnCoup.F(Veh(veh_num).global_ind) = Veh(veh_num).SysM.M * (Veh(veh_num).DOF.vert*Calc.Cte.grav);
    
end % for veh_num = 1:Veh(1).Tnum

% Track contribution
UnCoup.Kg(Veh(end).global_ind(end)+1:end,Veh(end).global_ind(end)+1:end) = Model.Mesh.Kg;
UnCoup.Cg(Veh(end).global_ind(end)+1:end,Veh(end).global_ind(end)+1:end) = Model.Mesh.Cg;
UnCoup.Mg(Veh(end).global_ind(end)+1:end,Veh(end).global_ind(end)+1:end) = Model.Mesh.Mg;

% % Symmetry check
% full([sum(sum(abs(UnCoup.Kg-UnCoup.Kg')))==0,sum(sum(abs(UnCoup.Cg-UnCoup.Cg')))==0,sum(sum(abs(UnCoup.Mg-UnCoup.Mg')))==0])

% **** With VBI ***
if Calc.Options.VBI == 1

    % Time Step Loop
    for t = 1:Calc.Solver.num_t-1

        % Progress display
        Aux = F01_Progress_Display(Aux,t,Calc.Solver.num_t);

        % ---- Time dependant system matrices ----
        Coup.Kg = UnCoup.Kg;
        Coup.Cg = UnCoup.Cg;
        Coup.Mg = UnCoup.Mg;
        Coup.F = UnCoup.F;

        % Vehicles contributions
        for veh_num = 1:Veh(1).Tnum

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
                    shape_fun_at_x_p = Track.Rail.Mesh.shape_fun_p(x,a);
                    shape_fun_at_x_pp = Track.Rail.Mesh.shape_fun_pp(x,a);
                    % Auxiliary matrices
                    NN = shape_fun_at_x*shape_fun_at_x';
                    NNp = shape_fun_at_x*shape_fun_at_x_p';
                    NNpp = shape_fun_at_x*shape_fun_at_x_pp';
                    % Addition primary suspension porperties to Track
                    eq_num = Veh(end).global_ind(end)+Track.Rail.Mesh.Ele.DOF(ele_num,:);
                    Coup.Kg(eq_num,eq_num) = Coup.Kg(eq_num,eq_num) + NN*Veh(veh_num).Susp.Prim.k(wheel) + ...
                        Veh(veh_num).Susp.Prim.c(wheel)*Train.vel*NNp + ...
                        Veh(veh_num).Wheels.m(wheel)*Train.vel^2*NNpp;
                    Coup.Cg(eq_num,eq_num) = Coup.Cg(eq_num,eq_num) + NN*Veh(veh_num).Susp.Prim.c(wheel) + ...
                        2*Veh(veh_num).Wheels.m(wheel)*Train.vel*NNp;

                    % Addition wheel masses to Track
                    Coup.Mg(eq_num,eq_num) = Coup.Mg(eq_num,eq_num) + NN*Veh(veh_num).Wheels.m(wheel);

                    % Off-diagonal block matrices
                    % Vehicle's Node to wheel displacements
                    N2w = Veh(veh_num).Wheels.N2w(wheel,:);
                    % Off diagonal block matrix
                    OffDiagBlockMat = -(shape_fun_at_x*N2w);
                    OffDiagBlockMat_d = -(shape_fun_at_x_p*N2w)*Train.vel;

                    % Addition to Coupled stiffness matrix
                    rows = Veh(veh_num).global_ind;
                    cols = Veh(end).global_ind(end)+Track.Rail.Mesh.Ele.DOF(ele_num,:);
                    Coup.Kg(rows,cols) = Coup.Kg(rows,cols) + OffDiagBlockMat'*Veh(veh_num).Susp.Prim.k(wheel) + ...
                        OffDiagBlockMat_d'*Veh(veh_num).Susp.Prim.c(wheel);
                    Coup.Kg(cols,rows) = Coup.Kg(cols,rows) + OffDiagBlockMat*Veh(veh_num).Susp.Prim.k(wheel);
                    % Addition to Coupled damping matrix
                    Coup.Cg(rows,cols) = Coup.Cg(rows,cols) + OffDiagBlockMat'*Veh(veh_num).Susp.Prim.c(wheel);
                    Coup.Cg(cols,rows) = Coup.Cg(cols,rows) + OffDiagBlockMat*Veh(veh_num).Susp.Prim.c(wheel);

                    % Force vector
                    Coup.F(cols) = (Veh(veh_num).Wheels.m(wheel)*Calc.Cte.grav + ...
                        -Veh(veh_num).Wheels.m(wheel)*Calc.Veh(veh_num).hdd_path(wheel,t+1))*shape_fun_at_x;
                    % Note that the velocity is already accounted for in hdd_path, which is time derivative

                    % Addition of profile
                    Coup.F(rows) = Coup.F(rows) + (Veh(veh_num).Susp.Prim.k(wheel)*Calc.Veh(veh_num).h_path(wheel,t+1) + ...
                        Veh(veh_num).Susp.Prim.c(wheel)*Calc.Veh(veh_num).hd_path(wheel,t+1))*N2w';
                    Coup.F(cols) = Coup.F(cols) - (Veh(veh_num).Susp.Prim.k(wheel)*Calc.Veh(veh_num).h_path(wheel,t+1) + ...
                        Veh(veh_num).Susp.Prim.c(wheel)*Calc.Veh(veh_num).hd_path(wheel,t+1))*shape_fun_at_x;
                end % if ele_num > 0
            end % for wheel = 1:Veh(veh_num).Wheels.num

        end % for veh_num = 1:Veh(1).Tnum

        % Re-apply the boundary conditions
        Coup.Mg(Coup.BC.DOF_fixed,:) = 0; Coup.Mg(:,Coup.BC.DOF_fixed) = 0;
        Coup.Cg(Coup.BC.DOF_fixed,:) = 0; Coup.Cg(:,Coup.BC.DOF_fixed) = 0;
        Coup.Kg(Coup.BC.DOF_fixed,:) = 0; Coup.Kg(:,Coup.BC.DOF_fixed) = 0;
        for i = 1:Coup.BC.num_DOF_fixed
            Coup.Mg(Coup.BC.DOF_fixed(i),Coup.BC.DOF_fixed(i)) = Model.BC.DOF_fixed_value;
            Coup.Kg(Coup.BC.DOF_fixed(i),Coup.BC.DOF_fixed(i)) = Model.BC.DOF_fixed_value;
        end % for i = 1:Coup.BC.num_DOF_fixed
        Coup.F(Coup.BC.DOF_fixed) = 0;

        % % Symmetry?
        %full([sum(sum(abs(Coup.Kg-Coup.Kg')))==0,sum(sum(abs(Coup.Cg-Coup.Cg')))==0,sum(sum(abs(Coup.Mg-Coup.Mg')))==0])

        % ---- Direct integraion ----

        % -- Newmark-Beta --
        % Effective Stiffness Matrix
        effKg = Coup.Kg + Coup.Mg/(Calc.Solver.NewMark_beta*Calc.Solver.dt^2) + ...
            Calc.Solver.NewMark_delta/(Calc.Solver.NewMark_beta*Calc.Solver.dt)*Coup.Cg;
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
                    % Force matrix elements
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
Sol.Model.Nodal.U = Coup.U(Veh(end).global_ind(end)+1:end,:);
Sol.Model.Nodal.V = Coup.V(Veh(end).global_ind(end)+1:end,:);
Sol.Model.Nodal.A = Coup.A(Veh(end).global_ind(end)+1:end,:);

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

end % function [Sol] = B65_DynamicCalcCoupled(Veh,Model,Calc,Track,Sol)

% ---- End of script ----
