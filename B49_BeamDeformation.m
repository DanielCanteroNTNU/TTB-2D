function [Sol] = B49_BeamDeformation(Sol,Model,Beam,Calc,Train,calc_type)

% Calculates beam deformation minimum from the nodal displacements.

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
% Train = Structure with Train variables, including at least:
%   .Veh(1).Tnum = Total number of vehicles
% calc_type = Flag to define what load effect to calculate:
%   0 = Static deformation
%   1 = Deformation
% ---- Outputs ----
% Sol = Addition of fields to structure Sol:
%   .Beam.(out_field).xt = Deformation for each node in time [Beam.Tnum_nodes x Calc.Solver.num_t]
%   .Beam.(out_field).min = Minimum deformation(whole length)
%   .Beam.(out_field).COP = Critical observation point
%   .Beam.(out_field).pCOP = Critical observation point in percentage of beam length
%   .Beam.(out_field).min05 = Minimum deformation at mid-span
% -------------------------------------------------------------------------

if calc_type == 0

    usefield = 'StaticU';

    % Vehicle loop
    for veh_num = 1:Train.Veh(1).Tnum

        % Definition of static force in time
        Calc.Veh(veh_num).F_onBeam = Train.Veh(veh_num).sta_loads*ones(1,Calc.Solver.num_t);
        % (NOTE = Calc.F is changed here. But because it is not an output the
        % change affects only wihtin this function)

    end % for veh_num = 1:Train.Veh(1).Tnum

    % Nodal forces calculation
    Model.Mesh.shape_fun = Beam.Mesh.shape_fun;
    [F] = B14_EqVertNodalForce(Model,Calc);
    
    % Nodal displacements
    Sol.Model.Nodal.(usefield) = Model.Mesh.Kg\full(F);

elseif calc_type == 1

    usefield = 'U';
    
end % if calc_type == 0

% Displacements
Sol.Beam.(usefield).xt = Sol.Model.Nodal.(usefield)(Model.Mesh.DOF.beam_vert,:);

% Note: Normal traffic vertical loading gives negative displacement. 
% Thus the minimum displacement is of interest

% ---- Additional Outputs ----
% Minimum Displacements
[Sol.Beam.(usefield).min,aux1] = min(Sol.Beam.(usefield).xt);
[Sol.Beam.(usefield).min,aux2] = min(Sol.Beam.(usefield).min);
Sol.Beam.(usefield).COP = Beam.Mesh.Nodes.acum(aux1(aux2));
Sol.Beam.(usefield).pCOP = Sol.Beam.(usefield).COP/Beam.Prop.L*100;
Sol.Beam.(usefield).t_crit = Calc.Solver.t(aux2);

% Mid-span Minimum Displacement
if Beam.Mesh.Nodes.Mid.exists == 1
    Sol.Beam.(usefield).min05 = min(Sol.Beam.(usefield).xt(Beam.Mesh.Nodes.Mid.node,:));
else
    aux1 = min(Sol.Beam.(usefield).xt,[],2);
    Sol.Beam.(usefield).min05 = interp1(Beam.Mesh.Nodes.acum,aux1,Beam.Prop.L/2);
end % if Beam.Mesh.Nodes.Mid.exists == 1

% ---- End of function ----
