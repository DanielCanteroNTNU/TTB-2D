function [Model] = B55_ModelBC(Model,Beam,Track)

% Applies the boundary conditions to the coupled model

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
% Model = Structure with Model variables, including at least:
%   ... see script ...
% Beam = Structure with Beam variables, including at least:
%   ... see script ...
% Track = Structure with Track variables, including at least:
%   ... see script ...
% % ---- Outputs ----
% Model = Additional fields to Model structure variable
%   ... see script ...
% -------------------------------------------------------------------------

% From Beam DOF to Model DOF
Model.BC.DOF_fixed = Model.Mesh.DOF.beam(Beam.BC.DOF_fixed);

% From Rail DOF to Model DOF
Model.BC.DOF_fixed = [Model.BC.DOF_fixed,...
    Model.Mesh.DOF.rail(Track.Rail.BC.DOF_fixed)];

% Sorting array
Model.BC.DOF_fixed = sort(Model.BC.DOF_fixed);

% Number of fixed DOF
Model.BC.num_DOF_fixed = length(Model.BC.DOF_fixed);

% Value to use in the diagonal element when the DOF is fixed
Model.BC.DOF_fixed_value = Beam.BC.DOF_fixed_value;

% Fixed DOF
Model.Mesh.Mg(Model.BC.DOF_fixed,:) = 0;
Model.Mesh.Mg(:,Model.BC.DOF_fixed) = 0;
Model.Mesh.Cg(Model.BC.DOF_fixed,:) = 0; 
Model.Mesh.Cg(:,Model.BC.DOF_fixed) = 0;
Model.Mesh.Kg(Model.BC.DOF_fixed,:) = 0; 
Model.Mesh.Kg(:,Model.BC.DOF_fixed) = 0;
for i = 1:Model.BC.num_DOF_fixed
    Model.Mesh.Mg(Model.BC.DOF_fixed(i),Model.BC.DOF_fixed(i)) = ...
        Model.BC.DOF_fixed_value;
    Model.Mesh.Kg(Model.BC.DOF_fixed(i),Model.BC.DOF_fixed(i)) = ...
        Model.BC.DOF_fixed_value;
end % for i = 1:Model.BC.num_DOF_fixed

% ---- End of function ----
