function [Sol] = B58_ResultsBeamSections(Sol,Beam,Calc)

% Calculates the available Beam results at predefined sections

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
% Sol = Structure with Sol variables, including at least:
%   ... see script ...
% Beam = Structure with Beam variables, including at least:
%   ... see script ...
% Calc = Structure with Calc variables, including at least:
%   ... see script ...
% % ---- Outputs ----
% Sol = Additional fields to Sol structure variable
%   ... see script ...
% -------------------------------------------------------------------------

if Calc.Options.num_calc_beam_sections > 0
    
    % Getting solutions names
    usefields = fields(Sol.Beam);

    % Auxiliary variables
    num_fields = size(usefields,1);

    % Fields loop
    for field = 1:num_fields

        aux1 = [];
        % Section loop
        for section = 1:Calc.Options.num_calc_beam_sections
            aux1(section,:) = interp1(Beam.Mesh.Nodes.acum,...
                Sol.Beam.(usefields{field}).xt,Calc.Options.calc_beam_sections(section));
        end % for section = 1:Calc.Options.num_calc_beam_sections
        
        Sol.Beam.(usefields{field}).sections_t = aux1;

    end % for field = 1:num_fields

end % if num_sections > 0

% ---- End of function ----
