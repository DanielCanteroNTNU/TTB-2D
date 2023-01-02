function [Sol] = B33_BeamShear(Sol,Model,Beam,Calc,calc_type)

% Calculates the Shear of the beam using the nodal displacements

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
% Sol = Structure with beam Solutions variables. It should include at least:
%   .Beam.U.xt = Nodal displacements of the beam FEM
%   .Beam.StaticU.xt = Static nodal displacements of the beam FEM
% Model = Structure with Model information. It should include at least:
%   ... see script ...
% Beam = Structure with Beam variables, including at least:
%   ... see script ...
% Calc = Structure with Calculation variables, including at least:
%   .Solver.num_t = Number of time steps for beam calculations
%   .Options.Shear_calc_mode = To select the mode of shear calculation
%       0 = Calculations done node by node
%       1 = Average results at the node 
% calc_type = Flag to define what load effect to calculate:
%   0 = Static Shear
%   1 = Shear
% ---- Outputs ----
% Sol = Addition of fields to structure Sol:
%   .Beam.(out_field).xt = Shear for each node in time [Beam.Mesh.Nodes.Tnum x Calc.Solver.num_t]
%   .Beam.(out_field).max = Maximum shear (whole length)
%   .Beam.(out_field).max_COP = Critical observation point of Maximum
%   .Beam.(out_field).max_pCOP = Critical observation point in percentage of beam length of Maximum
%   .Beam.(out_field).min = Minimum shear (whole length)
%   .Beam.(out_field).min_COP = Critical observation point of Minimum
%   .Beam.(out_field).min_pCOP = Critical observation point in percentage of beam length of Minimum
% -------------------------------------------------------------------------

% Input processing
if calc_type == 0
    out_field = 'StaticShear';
    in_field = 'StaticU';
elseif calc_type == 1
    out_field = 'Shear';
    in_field = 'U';
end % if calc_type == 0
    
% Initialize variables
Sol.Beam.(out_field).xt = zeros(Beam.Mesh.Nodes.Tnum,Calc.Solver.num_t);

% In-line functions (more efficient alternative to subfunctions)
B32_Beam_ele_HS = ...
    @(L,E,I) double(E*I*[[12/L^3,6/L^2,-12/L^3,6/L^2];[12/L^3,6/L^2,-12/L^3,6/L^2]]);

% ---- NO average nodal values ----
if Calc.Options.Shear_calc_mode == 0
    
    for ele = 1:Beam.Mesh.Ele.Tnum

        aux1 = B32_Beam_ele_HS(Beam.Mesh.Ele.a(ele),Beam.Prop.E_n(ele),Beam.Prop.I_n(ele));
        Sol.Beam.(out_field).xt(ele,:) = aux1(1,:) * ...
            Sol.Model.Nodal.U(Model.Mesh.DOF.beam(Beam.Mesh.Ele.DOF(ele,:)),:);

    end %for ele

    ele = Beam.Mesh.Nodes.Tnum;
    aux1 = B32_Beam_ele_HS(Beam.Mesh.Ele.a(ele-1),Beam.Prop.E_n(ele-1),Beam.Prop.I_n(ele-1));
    Sol.Beam.(out_field).xt(ele,:) = aux1(2,:) * ...
        Sol.Model.Nodal.(in_field)(Model.Mesh.DOF.beam(Beam.Mesh.Ele.DOF(ele-1,:)),:);
    
% ---- AVERAGE nodal values ----
elseif Calc.Options.Shear_calc_mode == 1
    
    for ele = 1:Beam.Mesh.Ele.Tnum

        Sol.Beam.(out_field).xt([1,2]+(ele-1),:) = Sol.Beam.(out_field).xt([1,2]+(ele-1),:) + ...
            B32_Beam_ele_HS(Beam.Mesh.Ele.a(ele),Beam.Prop.E_n(ele),Beam.Prop.I_n(ele)) * ... 
            Sol.Model.Nodal.(in_field)(Model.Mesh.DOF.beam(Beam.Mesh.Ele.DOF(ele,:)),:);

    end %for ele

    % Average of nodes with multiple calculations
    Sol.Beam.(out_field).xt(2:end-1,:) = Sol.Beam.(out_field).xt(2:end-1,:)/2;
    
end % Calc.Options.Shear_calc_mode

% ---- Additional Outputs ----

% Maximum Shear Force
[Sol.Beam.(out_field).max,aux1] = max(Sol.Beam.(out_field).xt);
[Sol.Beam.(out_field).max,aux2] = max(Sol.Beam.(out_field).max);
Sol.Beam.(out_field).max_node = aux1(aux2);
Sol.Beam.(out_field).max_COP = Beam.Mesh.Nodes.acum(Sol.Beam.(out_field).max_node);
Sol.Beam.(out_field).max_pCOP = Sol.Beam.(out_field).max_COP/Beam.Prop.L*100;
Sol.Beam.(out_field).max_t_crit = Calc.Solver.t(aux2);
if Sol.Beam.(out_field).max_pCOP < 50
    Sol.Beam.(out_field).max_supp = max(Sol.Beam.(out_field).xt(1,:));
else
    Sol.Beam.(out_field).max_supp = max(Sol.Beam.(out_field).xt(end,:));
end % if Sol.Beam.(out_field).max_pCOP < 50

% Minimum Shear Force
[Sol.Beam.(out_field).min,aux1] = min(Sol.Beam.(out_field).xt);
[Sol.Beam.(out_field).min,aux2] = min(Sol.Beam.(out_field).min);
Sol.Beam.(out_field).min_node = aux1(aux2);
Sol.Beam.(out_field).min_COP = Beam.Mesh.Nodes.acum(Sol.Beam.(out_field).min_node);
Sol.Beam.(out_field).min_pCOP = Sol.Beam.(out_field).min_COP/Beam.Prop.L*100;
Sol.Beam.(out_field).min_t_crit = Calc.Solver.t(aux2);
if Sol.Beam.(out_field).min_pCOP < 50
    Sol.Beam.(out_field).min_supp = min(Sol.Beam.(out_field).xt(1,:));
else
    Sol.Beam.(out_field).min_supp = min(Sol.Beam.(out_field).xt(end,:));
end % if Sol.Beam.(out_field).min_pCOP < 50

% ---- End of function ----
