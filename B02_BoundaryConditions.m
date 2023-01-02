function [Beam] = B02_BoundaryConditions(Beam)

% Definition of DOF with boundary conditions for different configurations

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
%   .BC.loc = location of supports in X direction
%   .BC.vert_stiff = Vertical stiffnes value of each of the supports
%       -1 =    Fixed, no displacement
%       0 =     Free vertical displacement
%       value = Vertical stiffness of the support
%   .BC.rot_stiff = Rotational stiffnes value of each of the supports
%       -1 =    Fixed, no rotation
%       0 =     Free rotation
%       value = Rotational stiffness of the support
% % ---- Outputs ----
% Beam = Addition of fields to structure Beam:
%   .BC.supp_num = Number of supports
%   .BC.loc_ind = Node number closest to the support
%   .BC.DOF_fixed = Array of DOF with fixed boundary condition
%   .BC.DOF_with_values = Array of DOF that have some additional stiffness
%   .BC.DOF_stiff_values = Array of stiffness values to be added to Beam.BC.DOF_with_values
%   .BC.num_DOF_fixed = Number of fixed DOF
%   .BC.num_DOF_with_values = Number of values with additional stiffness
%   .num_rigid_modes = Number of rigid modes
% -------------------------------------------------------------------------

% Number of supports
Beam.BC.supp_num = length(Beam.BC.loc);

% Supports location index
if Beam.BC.supp_num > 0
    [~,Beam.BC.loc_ind] = ...
        min(abs(ones(Beam.BC.supp_num,1)*Beam.Mesh.Nodes.acum - ...
        Beam.BC.loc'*ones(1,Beam.Mesh.Nodes.Tnum)),[],2);
else
    Beam.BC.loc_ind = [];
end % if Beam.BC.supp_num > 0
Beam.BC.loc_ind = Beam.BC.loc_ind';

% Fixed vertical displacement DOF
Beam.BC.DOF_fixed = Beam.BC.loc_ind(Beam.BC.vert_stiff==-1)*2-1;

% Fixed rotational DOF
Beam.BC.DOF_fixed = [Beam.BC.DOF_fixed, ...
    Beam.BC.loc_ind(Beam.BC.rot_stiff==-1)*2];

% Sorting fixed DOF
Beam.BC.DOF_fixed = sort(Beam.BC.DOF_fixed);

% Vertical displacement DOF with stiffness values
Beam.BC.DOF_with_values = Beam.BC.loc_ind(Beam.BC.vert_stiff>0)*2-1;
Beam.BC.DOF_stiff_values = Beam.BC.vert_stiff(Beam.BC.vert_stiff>0);

% Fixed rotational DOF
Beam.BC.DOF_with_values = [Beam.BC.DOF_with_values, ...
    Beam.BC.loc_ind(Beam.BC.rot_stiff>0)*2];
Beam.BC.DOF_stiff_values = [Beam.BC.DOF_stiff_values, ...
    Beam.BC.rot_stiff(Beam.BC.rot_stiff>0)];

% Sorting fixed DOF
[Beam.BC.DOF_with_values,aux2] = sort(Beam.BC.DOF_with_values);
Beam.BC.DOF_stiff_values = Beam.BC.DOF_stiff_values(aux2);

% Auxiliary variables
Beam.BC.num_DOF_fixed = length(Beam.BC.DOF_fixed);
Beam.BC.num_DOF_with_values = length(Beam.BC.DOF_with_values);
Beam.Modal.num_rigid_modes = max([0,2 - Beam.BC.num_DOF_fixed]);

% Value to use in the diagonal element when the DOF is fixed
% Usually the value 1 was used, but this gave some ill-conditioning in some cases
Beam.BC.DOF_fixed_value = 1e8;
%Beam.BC.DOF_fixed_value = 1;

% ---- End of script ----
