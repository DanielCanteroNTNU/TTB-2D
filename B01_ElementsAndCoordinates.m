function [Beam] = B01_ElementsAndCoordinates(Beam,varargin)

% Calculation of node coordinates and asociated DOF to each element.
% Specifies the properties element by element for (E, I, rho and A).
% Also generates additional and auxiliary variables needed in the FEM model.

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
%   .Prop.L = Bridge length [m]
%   .Mesh.Ele.num = Number of elements
%	.Prop.E = Material Young's Modulus [N/m^2]
%   .Prop.I = Section second moment of area [m^4]
%   .Prop.rho = Material density [kg/m^3]
%   .Prop.A = Section Area [m^2]
% % ---- Optional inputs ----
% Note: This input is only needed if information about mid-span location is needed
% Calc = Structure with Calc's strucutre variables, including at least:
%   .Cte.tol = numerical tolerance
% % ---- Outputs ----
% Beam = Addition of fields to structure Beam:
%   .Mesh.Ele.a = Vector with each element X dimension
%   .Mesh.Nodes.acum = X coordinate of each node
%   .Mesh.Nodes.coord = Coordinates of all nodes [X coord], one row for each node
%   .Mesh.Ele.num_nodes = Nodes per element
%   .Mesh.Ele.Nodes.num_DOF = DOF per node
%   .Mesh.Ele.Tnum = Total number of elements in the model
%   .Mesh.Ele.nodes = Each row includes the indices of the nodes for each element. 
%   .Mesh.Ele.DOF = Each row includes the DOF asociated to every element.
%   .Mesh.Nodes.Tnum = Total number of nodes
%   .Mesh.DOF.Tnum = Total number of DOFs
%   .Prop.E_n = Beam Young's Modulus
%       Array of values giving the E for each element (same size as Beam.a)
%   .Prop.I_n = Beam's section Second moment of area
%       Array of values giving the I for each element (same size as Beam.a)
%   .Prop.rho_n = Beam's density
%       Array of values giving the rho for each element (same size as Beam.a)
%   .Prop.A_n = Beam's section Area
%       Array of values giving the A for each element (same size as Beam.a)
% % ---- Optional Output ----
% Beam.Mesh.Nodes.Mid.exists = Flag indicating existence of node at mid-span
% Beam.Mesh.Nodes.Mid.node = Node number at mid-span
% Beam.Mesh.DOF.Mid.vert = Vertical DOF at mid-span
% -------------------------------------------------------------------------

% Mesh definition 
Beam.Mesh.Ele.a = ones(1,Beam.Mesh.Ele.num)*Beam.Prop.L/Beam.Mesh.Ele.num;
Beam.Mesh.Nodes.acum = [0 cumsum(Beam.Mesh.Ele.a)];

% -- Coordenates for each node (nodes_coord) --
Beam.Mesh.Nodes.coord = Beam.Mesh.Nodes.acum';
Beam.Mesh.Ele.num_nodes = 2;
Beam.Mesh.Nodes.num_DOF = 2;
Beam.Mesh.Ele.Tnum = Beam.Mesh.Ele.num;
Beam.Mesh.Ele.nodes = [(1:Beam.Mesh.Ele.num);(1:Beam.Mesh.Ele.num)+1]';
Beam.Mesh.Ele.DOF = (1:2:Beam.Mesh.Ele.Tnum*Beam.Mesh.Ele.num_nodes)';
Beam.Mesh.Ele.DOF = [Beam.Mesh.Ele.DOF,Beam.Mesh.Ele.DOF+1,...
    Beam.Mesh.Ele.DOF+2,Beam.Mesh.Ele.DOF+3];
Beam.Mesh.Nodes.Tnum = Beam.Mesh.Ele.Tnum+1;
Beam.Mesh.DOF.Tnum = Beam.Mesh.Nodes.Tnum*Beam.Mesh.Nodes.num_DOF;

% -- Element by element property definition Input processing --
% Beam Young's Modulus
if length(Beam.Prop.E) == 1
    Beam.Prop.E_n = ones(Beam.Mesh.Ele.Tnum,1)*Beam.Prop.E;
end % if length(Beam.Prop.E) == 1
% Beam's section Second moment of Inertia product
if length(Beam.Prop.I) == 1
    Beam.Prop.I_n = ones(Beam.Mesh.Ele.Tnum,1)*Beam.Prop.I;
end % if length(Beam.Prop.I) == 1
% Beam's density
if length(Beam.Prop.rho) == 1
    Beam.Prop.rho_n = ones(Beam.Mesh.Ele.Tnum,1)*Beam.Prop.rho;
end % if length(Beam.I) == 1
% Beam's section Area
if length(Beam.Prop.A) == 1
    Beam.Prop.A_n = ones(Beam.Mesh.Ele.Tnum,1)*Beam.Prop.A;
end % if length(Beam.A) == 1

% ---- Optional calculations ----
% Note: Only performed if a second input is defined

if nargin == 2
    
    Calc = varargin{1};

    % -- Mid-span information --
    [min_value,min_ind] = min(abs(Beam.Mesh.Nodes.acum-Beam.Prop.L/2));
    if min_value < Calc.Cte.tol
        Beam.Mesh.Nodes.Mid.exists = 1;
        Beam.Mesh.Nodes.Mid.node = min_ind;
        Beam.Mesh.DOF.Mid.vert = ...
            Beam.Mesh.Nodes.num_DOF*Beam.Mesh.Nodes.Mid.node - 1;
    else
        Beam.Mesh.Nodes.Mid.exists = 0;
    end % if min_value < Cacl.Cte.tol

end % if nargin == 2

% ---- End of function ----
