function [Beam] = B24_BeamDamping(Beam)

% Calculates the Beam damping matrix 
%   Rayleigh damping is addopted 
%   1st and 2nd beam frequencies are taken as reference (excluding rigid modes)

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
%   .Modal.w = Beam circular frequencies
%   .Modal.num_rigid_modes = Number of rigid modes in the model
%   .Damping.per = Damping percentage
%   .Mesh.Mg = Mass matrix of beam
%   .Mesh.Kg = Stiffness matrix of beam
% % ---- Outputs ----
% Beam = Addition of fields to structure Beam:
%   .Cg = Global Damping matrix
% -------------------------------------------------------------------------

if Beam.Damping.per > 0
    
    % Reference frequencies
    wr = Beam.Modal.w((1:2)+Beam.Modal.num_rigid_modes);
    
    % Rayleigh's coefficients 'alpha' and 'beta'
    aux1 = (1/2)*[[1/wr(1) wr(1)];[1/wr(2) wr(2)]]\([1;1]*(Beam.Damping.per/100));

    % Damping matrix
    Beam.Mesh.Cg = aux1(1)*Beam.Mesh.Mg + aux1(2)*Beam.Mesh.Kg;
    
%     % Graphical check
%     w = linspace(wr(1)/2,wr(2)*1.5,100);
%     figure; hold on;
%         plot(w,aux1(1)./(2*w)+aux1(2)*w/2);
%         plot(wr,Beam.Damping.per/100*[1,1],'r.');
%         xlabel('Circular frequency'); ylabel('Damping ratio');

else
    
    % No Damping case
    Beam.Mesh.Cg = Beam.Mesh.Kg*0;
    
end % if Beam.Damping.per > 0

% ---- End of script ----
