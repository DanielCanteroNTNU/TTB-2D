function [Model] = B56_ModelFrq(Model,Calc)

% Adapted version of B09_BeamFrq
% Calculates the model modes and frequencies given the system matrices

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
% ---- Input ----
% Model = Structure with Model's variables, including at least:
%   .Mesh.Kg = Global Stiffness matrix
%   .Mesh.Mg = Global Mass matrix
%   .BC.num_DOF_fixed = Number of fixed DOF
% Calc = Structre with calculation variables. It should include at least:
%   .Options.calc_model_frq = If value = 1, model's natural frequencies are calculated
%   .Options.calc_model_modes = If value = 1, model's modes of vibration are calculated
% ---- Output ----
% Model = Addition of fields to structure Model:
%   .Modal.w = Circular frequencies of the Model
%   .Modal.f = Natural frequencies of the Model
%   .Modal.modes = Modes of vibration of in columns
% -------------------------------------------------------------------------

% Only natural frequencies calculation
if and(Calc.Options.calc_model_frq == 1,Calc.Options.calc_model_modes == 0)
    
    disp('Calculating model frequencies ...')
    
    lambda = eig(full(Model.Mesh.Kg),full(Model.Mesh.Mg));
    Model.Modal.w = sqrt(lambda);
    Model.Modal.f = Model.Modal.w/(2*pi);

    % Removing values associated to BC
    Model.Modal.w = Model.Modal.w(Model.BC.num_DOF_fixed+1:end);
    Model.Modal.f = Model.Modal.f(Model.BC.num_DOF_fixed+1:end);
    
% Natural frequencies and Modes of vibration calculation
elseif and(Calc.Options.calc_model_frq == 1,Calc.Options.calc_model_modes == 1)
    
    disp('Calculating model modes and frequencies ...')
    
    [V,lambda] = eig(full(Model.Mesh.Kg),full(Model.Mesh.Mg));
    [lambda,k] = sort(diag(lambda));
    V = V(:,k); 
    
    % Normaliztion of eigenvectors
    Factor = diag(V'*Model.Mesh.Mg*V); 
    Model.Modal.modes = V/(sqrt(diag(Factor)));
    
    % EigenValues to Natural frequencies
    Model.Modal.w = sqrt(lambda);
    Model.Modal.f = Model.Modal.w/(2*pi);
    
    % Removing values associated to BC
    Model.Modal.w = Model.Modal.w(Model.BC.num_DOF_fixed+1:end);
    Model.Modal.f = Model.Modal.f(Model.BC.num_DOF_fixed+1:end);
    Model.Modal.modes(:,1:Model.BC.num_DOF_fixed) = [];
    
end % if and(Calc.Options.calc_model_frq == 1,Calc.Options.calc_model_frq == 0)

% ---- End of function ----
