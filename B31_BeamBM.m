function [Sol] = B31_BeamBM(Sol,Model,Beam,Calc,calc_type)

% Calculates the Bending Moment (BM) of the beam using the nodal displacements

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
% Sol = Structure with Solution variables. It should include at least:
%   .Beam.U.xt = Nodal displacements of the beam FEM
%   .Beam.StaticU.xt = Static nodal displacements of the beam FEM
% Model = Structure with Model information. It should include at least:
%   ... see script ...
% Beam = Structure with Beam variables, including at least:
%   ... see script ...
% Calc = Structure with Calculation variables, including at least:
%   .Solver.num_t = Number of time steps for beam calculations
%   .Options.BM_calc_mode = To select the mode of BM calculation
%       0 = Calculations done node by node
%       1 = Average results at the node 
% calc_type = Flag to define what load effect to calculate:
%   0 = Static BM
%   1 = BM
% ---- Outputs ----
% Sol = Addition of fields to structure Sol:
%   .Beam.(out_field).xt = Bending moment for each node in time [Beam.Tnum_nodes x Calc.Solver.num_t]
%   .Beam.(out_field).maxBM = Maximum BM (whole length)
%   .Beam.(out_field).COP = Critical observation point
%   .Beam.(out_field).pCOP = Critical observation point in percentage of beam length
%   .Beam.(out_field).minBM = Minimum Bending moment (whole length)
%   .Beam.(out_field).maxBM05 = Maximum Bending moment at mid-span
% -------------------------------------------------------------------------

% Input processing
if calc_type == 0
    out_field = 'StaticBM';
    in_field = 'StaticU';
elseif calc_type == 1
    out_field = 'BM';
    in_field = 'U';
end % if calc_type == 0

% Initialize variables
Sol.Beam.(out_field).xt = zeros(Beam.Mesh.Nodes.Tnum,Calc.Solver.num_t);

% In-line functions (more efficient alternative to subfunctions)
B30_Beam_ele_H = ...
    @(L,E,I) double(E*I*[[-6/L^2,-4/L,6/L^2,-2/L];[6/L^2,2/L,-6/L^2,4/L]]);

% ---- NO average nodal values ----
if Calc.Options.BM_calc_mode == 0
    
    for ele = 1:Beam.Mesh.Ele.Tnum

        aux1 = B30_Beam_ele_H(Beam.Mesh.Ele.a(ele),Beam.Prop.E_n(ele),Beam.Prop.I_n(ele));
        Sol.Beam.(out_field).xt(ele,:) = aux1(1,:) * ...
            Sol.Model.Nodal.(in_field)(Model.Mesh.DOF.beam(Beam.Mesh.Ele.DOF(ele,:)),:);

    end %for ele

    ele = Beam.Mesh.Nodes.Tnum;
    aux1 = B30_Beam_ele_H(Beam.Mesh.Ele.a(ele-1),Beam.Prop.E_n(ele-1),Beam.Prop.I_n(ele-1));
    Sol.Beam.(out_field).xt(ele,:) = aux1(2,:) * ...
        Sol.Model.Nodal.(in_field)(Model.Mesh.DOF.beam(Beam.Mesh.Ele.DOF(ele-1,:)),:);
    
% ---- AVERAGE nodal values ----
elseif Calc.Options.BM_calc_mode == 1
    
    for ele = 1:Beam.Mesh.Ele.Tnum

        Sol.Beam.(out_field).xt([1,2]+(ele-1),:) = Sol.Beam.(out_field).xt([1,2]+(ele-1),:) + ...
            B30_Beam_ele_H(Beam.Mesh.Ele.a(ele),Beam.Prop.E_n(ele),Beam.Prop.I_n(ele)) * ... 
            Sol.Model.Nodal.(in_field)(Model.Mesh.DOF.beam(Beam.Mesh.Ele.DOF(ele,:)),:);

    end % for ele

    % Average of nodes with multiple calculations
    Sol.Beam.(out_field).xt(2:end-1,:) = Sol.Beam.(out_field).xt(2:end-1,:)/2;
    
end % Calc.Options.BM_calc_mode

% ---- Additional Outputs ----

% Maximum Bending Moment
[Sol.Beam.(out_field).max,aux1] = max(Sol.Beam.(out_field).xt);
[Sol.Beam.(out_field).max,aux2] = max(Sol.Beam.(out_field).max);
Sol.Beam.(out_field).COP = Beam.Mesh.Nodes.acum(aux1(aux2));
Sol.Beam.(out_field).pCOP = Sol.Beam.(out_field).COP/Beam.Prop.L*100;
Sol.Beam.(out_field).t_crit = Calc.Solver.t(aux2);

% Mid-span Bending Moment
if Beam.Mesh.Nodes.Mid.exists == 1
    Sol.Beam.(out_field).max05 = max(Sol.Beam.(out_field).xt(Beam.Mesh.Nodes.Mid.node,:));
else
    aux1 = max(Sol.Beam.(out_field).xt,[],2);
    Sol.Beam.(out_field).max05 = interp1(Beam.Mesh.Nodes.acum,aux1,Beam.Prop.L/2);
end % if Beam.Mesh.Nodes.Mid.exists == 1

% Additional calculation for BM minima
for k = 1:2
    if k == 1
        field = 'LeftSup';
        ind_add = 0;
        ind = ind_add+1:floor(Beam.Mesh.Nodes.Tnum/2);
    elseif k == 2
        field = 'RightSup';
        ind_add = ceil(Beam.Mesh.Nodes.Tnum/2)-1;
        ind = ind_add+1:Beam.Mesh.Nodes.Tnum;
    end % if k == 1

    [Sol.Beam.(out_field).(field).min,aux1] = min(Sol.Beam.(out_field).xt(ind,:));
    [Sol.Beam.(out_field).(field).min,aux2] = min(Sol.Beam.(out_field).(field).min);
    Sol.Beam.(out_field).(field).COP = Beam.Mesh.Nodes.acum(aux1(aux2)+ind_add);
    Sol.Beam.(out_field).(field).pCOP = Sol.Beam.(out_field).(field).COP/Beam.Prop.L*100;
    Sol.Beam.(out_field).(field).min05 = min(Sol.Beam.(out_field).xt(1,:));
    
end % for k = 1:2

% ---- End of function ----
