function [Beam] = B03_BeamMatrices(Beam)

% Generates the FEM system matrices for the beam model

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
%   .Mesh.Ele.a = Vector with elements size on X direction
%   .Prop.E_n = Beam Young's Modulus
%       Array of values giving the E for each element (same size as Beam.a)
%   .Prop.I_n = Beam's section Second moment of Inertia product
%       Array of values giving the I for each element (same size as Beam.a)
%   .Prop.rho_n = Beam's density
%       Array of values giving the rho for each element (same size as Beam.a)
%   .Prop.A_n = Beam's section Area
%       Array of values giving the A for each element (same size as Beam.a)
%   .BC.DOF_fixed = Array of DOF with fixed boundary condition
%   .BC.DOF_with_values = Array of DOF that have some additional stiffness
%   .BC.DOF_stiff_values = Array of stiffness values to be added to Beam.BC.DOF_with_values
%   .BC.num_DOF_fixed = Number of fixed DOF
%   .BC.num_DOF_with_values = Number of values with additional stiffness
%   .Options.k_Mconsist = Option to select the type of Mass matrix
%       1 = Consitent mass matrix (Default)
%       2 = Lumped mass matrix
% % ---- Outputs ----
% Beam = Addition of fields to structure Beam:
%   .Mesh.Kg = Global Stiffness matrix
%   .Mesh.Mg = Global Mass matrix
% -------------------------------------------------------------------------

% Initialize matrices
Beam.Mesh.Kg = sparse(Beam.Mesh.DOF.Tnum,Beam.Mesh.DOF.Tnum);
CMM = Beam.Mesh.Kg;

% Elemental matrices (In-line functions) (more efficient alternative to subfunctions)
B04_Beam_ele_M = @(r,A,L) double(r*A*L/420*[[156,22*L,54,-13*L];[22*L,4*L^2,13*L,-3*L^2];...
    [54,13*L,156,-22*L];[-13*L,-3*L^2,-22*L,4*L^2]]);
B05_Beam_ele_K = @(EI,L) double(EI/L^3*[[12,6*L,-12,6*L];[6*L,4*L^2,-6*L,2*L^2];...
    [-12,-6*L,12,-6*L];[6*L,2*L^2,-6*L,4*L^2]]);

% ---- Beam element Shape function ----
% Beam.Mesh.shape_fun = @(x,a) [(a+2*x).*(a-x).^2./a.^3; x.*(a-x).^2./a.^2; x.^2.*(3*a-2*x)./a.^3; -x.^2.*(a-x)./a.^2];
% Beam.Mesh.shape_fun_p = @(x,a) [-(6*x.*(a - x))./a.^3; 1 - (x.*(4*a - 3*x))./a.^2; (6*x.*(a - x))./a.^3; -(x.*(2*a - 3*x))./a.^2];
% Beam.Mesh.shape_fun_pp = @(x,a) [(12*x)./a.^3 - 6./a.^2; (6*x)./a.^2 - 4./a; 6./a.^2 - (12*x)./a.^3; (6*x)./a.^2 - 2./a];
% - Simply adding the double() command makes it faster -
Beam.Mesh.shape_fun = @(x,a) double([(a+2*x).*(a-x).^2./a.^3; x.*(a-x).^2./a.^2; x.^2.*(3*a-2*x)./a.^3; -x.^2.*(a-x)./a.^2]);
Beam.Mesh.shape_fun_p = @(x,a) double([-(6*x.*(a - x))./a.^3; 1 - (x.*(4*a - 3*x))./a.^2; (6*x.*(a - x))./a.^3; -(x.*(2*a - 3*x))./a.^2]);
Beam.Mesh.shape_fun_pp = @(x,a) double([(12*x)./a.^3 - 6./a.^2; (6*x)./a.^2 - 4./a; 6./a.^2 - (12*x)./a.^3; (6*x)./a.^2 - 2./a]);

% ---- Stiffness and Consistent Mass Matrices ----
for ele_num = 1:Beam.Mesh.Ele.Tnum

    % Element matrices
    Me = B04_Beam_ele_M(Beam.Prop.rho_n(ele_num),Beam.Prop.A_n(ele_num),Beam.Mesh.Ele.a(ele_num));
    Ke = B05_Beam_ele_K(Beam.Prop.E_n(ele_num)*Beam.Prop.I_n(ele_num),Beam.Mesh.Ele.a(ele_num));
    
    % Assembly of matrices
    Beam.Mesh.Kg(Beam.Mesh.Ele.DOF(ele_num,:),Beam.Mesh.Ele.DOF(ele_num,:)) = ...
        Beam.Mesh.Kg(Beam.Mesh.Ele.DOF(ele_num,:),Beam.Mesh.Ele.DOF(ele_num,:)) + Ke;
    CMM(Beam.Mesh.Ele.DOF(ele_num,:),Beam.Mesh.Ele.DOF(ele_num,:)) = ...
        CMM(Beam.Mesh.Ele.DOF(ele_num,:),Beam.Mesh.Ele.DOF(ele_num,:)) + Me;

end %for ele_num = 1:Beam.Tnum_ele

if isfield(Beam.Prop,'m2')
    for ele_num = [1,Beam.Mesh.Ele.Tnum]
        % Element matrices
        Me = B04_Beam_ele_M(Beam.Prop.m2/Beam.Mesh.Ele.a(ele_num),1,Beam.Mesh.Ele.a(ele_num));
        % Assembly of matrices
        CMM(Beam.Mesh.Ele.DOF(ele_num,:),Beam.Mesh.Ele.DOF(ele_num,:)) = ...
            CMM(Beam.Mesh.Ele.DOF(ele_num,:),Beam.Mesh.Ele.DOF(ele_num,:)) + Me;
    end % for ele_num = [1,Beam.Mesh.Ele.Tnum]
end % if isfield(Beam.Prop,'m2')

if Beam.Options.k_Mconsist ~= 1

    % ---- Lumped Mass Matrix (LMM) ----
    % Initialize matrix
    LMM = Beam.Mesh.Kg*0;
    
    for ele_num = 1:Beam.Mesh.Ele.Tnum

        % Element matrix
        Me = Beam.Prop.rho_n(ele_num)*Beam.Prop.A_n(ele_num)*Beam.Mesh.Ele.a(ele_num)*diag([1/2,0,1/2,0]);
        
        % LMM assembly
        LMM(Beam.Mesh.Ele.DOF(ele_num,:),Beam.Mesh.Ele.DOF(ele_num,:)) = ...
            LMM(Beam.Mesh.Ele.DOF(ele_num,:),Beam.Mesh.Ele.DOF(ele_num,:)) + Me;

    end %for ele_num = 1:Beam.Tnum_ele

    % Mass Matrix
    Beam.Mesh.Mg = Beam.Options.k_Mconsist*CMM + (1-Beam.Options.k_Mconsist)*LMM;
    
else % if Beam.Options.k_Mconsist ~= 1

    Beam.Mesh.Mg = CMM;
    
end % if Beam.Options.k_Mconsist ~= 1

% ---- Application of boundary conditions ----
% Diagonal elements equal 1, and columns and rows equal zero for bc DOF

% Support DOF with values
for i = 1:Beam.BC.num_DOF_with_values
    Beam.Mesh.Kg(Beam.BC.DOF_with_values(i),Beam.BC.DOF_with_values(i)) = ...
        Beam.Mesh.Kg(Beam.BC.DOF_with_values(i),Beam.BC.DOF_with_values(i)) + ...
        Beam.BC.DOF_stiff_values(i);
end % for i = 1:Beam.BC.num_DOF_values

% Fixed DOF
Beam.Mesh.Kg(Beam.BC.DOF_fixed,:) = 0; Beam.Mesh.Kg(:,Beam.BC.DOF_fixed) = 0;
Beam.Mesh.Mg(Beam.BC.DOF_fixed,:) = 0; Beam.Mesh.Mg(:,Beam.BC.DOF_fixed) = 0;
for i = 1:Beam.BC.num_DOF_fixed
    Beam.Mesh.Kg(Beam.BC.DOF_fixed(i),Beam.BC.DOF_fixed(i)) = Beam.BC.DOF_fixed_value;
    Beam.Mesh.Mg(Beam.BC.DOF_fixed(i),Beam.BC.DOF_fixed(i)) = Beam.BC.DOF_fixed_value;
end % for i = 1:Beam.BC.num_DOF_fixed

% ---- End of function ----
