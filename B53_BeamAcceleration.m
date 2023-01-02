function [Sol] = B53_BeamAcceleration(Sol,Model,Beam,Calc)

% Extracts the Beam acceleration from the nodal displacements

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
% ---- Outputs ----
% Sol = Addition of fields to structure Sol:
%   .Beam.Acc.xt = Accelerations for each node in time [Beam.Tnum_nodes x Calc.Solver.num_t]
%   .Beam.Acc.max = Maximum acceleration (whole length)
%   .Beam.Acc.COP = Critical observation point
%   .Beam.Acc.pCOP = Critical observation point in percentage of beam length
%   .Beam.Acc.max05 = Maximum acceleration at mid-span
% -------------------------------------------------------------------------

% Displacements
Sol.Beam.Acc.xt = Sol.Model.Nodal.A(Model.Mesh.DOF.beam_vert,:);

% Note: The absolute value of the acceleration is analyzed

% ---- Additional Outputs ----
% Maximum Acceleration
[Sol.Beam.Acc.max,aux1] = max(abs(Sol.Beam.Acc.xt));
[Sol.Beam.Acc.max,aux2] = max(Sol.Beam.Acc.max);
Sol.Beam.Acc.COP = Beam.Mesh.Nodes.acum(aux1(aux2));
Sol.Beam.Acc.pCOP = Sol.Beam.Acc.COP/Beam.Prop.L*100;
Sol.Beam.Acc.t_crit = Calc.Solver.t(aux2);

% Mid-span Acceleration
if Beam.Mesh.Nodes.Mid.exists == 1
    Sol.Beam.Acc.max05 = max(abs(Sol.Beam.Acc.xt(Beam.Mesh.Nodes.Mid.node,:)));
else
    aux1 = max(abs(Sol.Beam.Acc.xt),[],2);
    Sol.Beam.Acc.max05 = interp1(Beam.Mesh.Nodes.acum,aux1,Beam.Prop.L/2);
end % if Beam.Mesh.Nodes.Mid.exists == 1

% ---- End of function ----
