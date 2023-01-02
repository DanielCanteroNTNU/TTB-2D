function [Veh] = B18_TrainVehEq(Veh)

% Generates the train vehicle equations for all the vehicles

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
% Veh = Indexed structure variable with information about each vehicle
%   of the Train. Include at least:
%   ... see script ...
% ---- Output ----
% Veh = Addtion of information to Veh structure:
%   .SysM = System matrices for each vehicle
%   .Tnum_DOF = Total number of DOF for each vehicle
%   .Wheels.N2w = Relation of nodal displacements to wheel displacements
%   .ktn = Array with primary suspension stiffness for each wheel
%   .ctn = Array with primary suspension viscous damping for each wheel
%   .mtn = Array with the masses for each wheel
%   .local_ind = Local indices for the DOFs
%   .global_ind = Global indices for the DOFs
%   .DOF = Description of the DOFs
%       .vert = Array indicating which ones are vertical displacement DOFs
%       .rot = Array indicating which ones are rotational DOFs
% -------------------------------------------------------------------------

% Vehicle loop
for veh_num = 1:Veh(1).Tnum

    % Variables definition
    m = Veh(veh_num).Body.m;
    I = Veh(veh_num).Body.I;
    mB1 = Veh(veh_num).Bogie.m(1);
    mB2 = Veh(veh_num).Bogie.m(2);
    IB1 = Veh(veh_num).Bogie.I(1);
    IB2 = Veh(veh_num).Bogie.I(2);
    k1 = Veh(veh_num).Susp.Prim.k(1);
    k2 = Veh(veh_num).Susp.Prim.k(2);
    k3 = Veh(veh_num).Susp.Prim.k(3);
    k4 = Veh(veh_num).Susp.Prim.k(4);
    c1 = Veh(veh_num).Susp.Prim.c(1);
    c2 = Veh(veh_num).Susp.Prim.c(2);
    c3 = Veh(veh_num).Susp.Prim.c(3);
    c4 = Veh(veh_num).Susp.Prim.c(4);
    ks1 = Veh(veh_num).Susp.Sec.k(1);
    ks2 = Veh(veh_num).Susp.Sec.k(2);
    cs1 = Veh(veh_num).Susp.Sec.c(1);
    cs2 = Veh(veh_num).Susp.Sec.c(2);
    dB1_F = Veh(veh_num).Bogie.L(1)/2;
    dB1_B = Veh(veh_num).Bogie.L(2)/2;
    dB2_F = Veh(veh_num).Bogie.L(1)/2;
    dB2_B = Veh(veh_num).Bogie.L(2)/2;
    if isfield(Veh(veh_num).Body,'L_F')
        if ~isempty(Veh(veh_num).Body.L_F)
            d_F = Veh(veh_num).Body.L_F;
            d_B = Veh(veh_num).Body.L_B;
        else
            d_F = Veh(veh_num).Body.L/2;
            d_B = Veh(veh_num).Body.L/2;
        end % if ~isempty(Veh(veh_num).Body.L_F)
    else
        d_F = Veh(veh_num).Body.L/2;
        d_B = Veh(veh_num).Body.L/2;
    end % if isfield(Veh.Body,'L_F')

    % Mass matrix
    Veh(veh_num).SysM.M = ...
        [[   m,   0,   0,   0,   0,   0]; ...
        [   0, mB1,   0,   0,   0,   0]; ...
        [   0,   0, mB2,   0,   0,   0]; ...
        [   0,   0,   0,   I,   0,   0]; ...
        [   0,   0,   0,   0, IB1,   0]; ...
        [   0,   0,   0,   0,   0, IB2]];

    % Damping matrix
    Veh(veh_num).SysM.C = ...
        [[               cs1+cs2,                  -cs1,                  -cs2,       cs1*d_F-cs2*d_B,                     0,                     0]; ...
        [                  -cs1,             c1+c2+cs1,                     0,              -cs1*d_F,     c1*dB1_F-c2*dB1_B,                     0]; ...
        [                  -cs2,                     0,             c3+c4+cs2,               cs2*d_B,                     0,     c3*dB2_F-c4*dB2_B]; ...
        [       d_F*cs1-cs2*d_B,              -d_F*cs1,               cs2*d_B,   cs1*d_F^2+cs2*d_B^2,                     0,                     0]; ...
        [                     0,     dB1_F*c1-dB1_B*c2,                     0,                     0, c1*dB1_F^2+c2*dB1_B^2,                     0]; ...
        [                     0,                     0,     dB2_F*c3-dB2_B*c4,                     0,                     0, c3*dB2_F^2+c4*dB2_B^2]];

    % Stiffness matrix
    Veh(veh_num).SysM.K = ...
        [[               ks1+ks2,                  -ks1,                  -ks2,       d_F*ks1-d_B*ks2,                     0,                     0]; ...
        [                  -ks1,             k1+k2+ks1,                     0,              -d_F*ks1,     dB1_F*k1-dB1_B*k2,                     0]; ...
        [                  -ks2,                     0,             k3+k4+ks2,               d_B*ks2,                     0,     dB2_F*k3-dB2_B*k4]; ...
        [       d_F*ks1-d_B*ks2,              -d_F*ks1,               d_B*ks2,   d_F^2*ks1+d_B^2*ks2,                     0,                     0]; ...
        [                     0,     dB1_F*k1-dB1_B*k2,                     0,                     0, k1*dB1_F^2+k2*dB1_B^2,                     0]; ...
        [                     0,                     0,     dB2_F*k3-dB2_B*k4,                     0,                     0, k3*dB2_F^2+k4*dB2_B^2]];

    % Total number of vehicle DOF
    Veh(veh_num).Tnum_DOF = size(Veh(veh_num).SysM.M,1);

    % ---- Wheel displacements relation ----
    % Nodal displacements to wheel displacements
    Veh(veh_num).Wheels.N2w = ...
        [[0,1,0,0,dB1_F,0]; ...
         [0,1,0,0,-dB1_B,0]; ...
         [0,0,1,0,0,dB2_F];
         [0,0,1,0,0,-dB2_B]];

    % Wheel mechanical properties
    Veh(veh_num).ktn = Veh(veh_num).Susp.Prim.k;
    Veh(veh_num).ctn = Veh(veh_num).Susp.Prim.c;
    Veh(veh_num).mtn = Veh(veh_num).Wheels.m;

    % Local and global indices for the DOF
    Veh(veh_num).local_ind = 1:Veh(veh_num).Tnum_DOF;
    if veh_num == 1
        Veh(veh_num).global_ind = Veh(veh_num).local_ind;
    else
        Veh(veh_num).global_ind = Veh(veh_num-1).global_ind(end) + Veh(veh_num).local_ind;
    end % if veh_num > 1

    % Description of the DOF
    Veh(veh_num).DOF.vert = [1;1;1;0;0;0];      % Vertical displacements DOF
    Veh(veh_num).DOF.rot = [0;0;0;1;1;1];       % Rotational DOF

end % for veh_num = 1:Veh(1).Tnum

% ---- End of script ----
