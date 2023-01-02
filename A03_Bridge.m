% Bridge model definition

% Not a function.
% Defines the content of the variable Beam.

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

% A 50m bridge. Properties taken from:
% He Xia, Nan Zhang, Guido De Roeck, Dynamic analysis of high speed railway 
%   bridge under articulated trains, Computers & Structures, Volume 81, 
%   Issues 26–27, 2003, Pages 2467-2478, ISSN 0045-7949,
%   https://doi.org/10.1016/S0045-7949(03)00309-2.
Beam.Prop.L = 50;       % Span [m]
Beam.Prop.E = 35e9;     % Modulus of elasticity [N/m^2]
Beam.Prop.I = 51.3;     % Second moment of area [m4]
Beam.Damping.per = 1;   % Damping [%]
Beam.Prop.rho = 69000;  % Mass per unit length [kg/m]

% Boundary conditions
Beam.BC.text = 'SP';    % Simply supported
%Beam.BC.text = 'FF';    % Fixed-fixed

% % Boundary conditions - Alternative definitions
% Beam.BC.loc = [0,Beam.Prop.L/2,Beam.Prop.L];  % Location of supports
% Beam.BC.vert_stiff = [-1,-1,-1];              % Vertical stiffness of supports (-1 for perfectly stiff)
% Beam.BC.rot_stiff = [0,0,0];                  % Rotational stiffness of supports (-1 for perfectly stiff)
% Beam.BC.text = 'User-defined';
% Beam.BC.text_long = '2-span bridge';

% ---- End of script ----
