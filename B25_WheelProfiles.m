function [Calc] = B25_WheelProfiles(Calc,Veh)

% Calculates the profile under each wheel

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
% Calc = Structure with Calculation variables. It should include at least:
%   ... see script ...
% Veh = Indexed Structre with Vehicle variables. It should include at least:
%   ... see script ...
% ---- Output ----
% Calc = Additional fields in the structure:
%   .Veh(i).x_path = Matrix with x coordinates for each wheel in time
%   .Veh(i).h_path = Matrix with irregularity elevation for each wheel in time
%   .Veh(i).hd_path = Matrix with 1st time-derivative of irregularity
%   .Veh(i).hdd_path = Matrix with 2nd time-derivative of irregularity
% -------------------------------------------------------------------------

for veh_num = 1:Veh(1).Tnum
    
    % Initialize variables
    Calc.Veh(veh_num).x_path = zeros(Veh(veh_num).Wheels.num,Calc.Solver.num_t);
    Calc.Veh(veh_num).h_path = Calc.Veh(veh_num).x_path;

    % Profile for each wheel
    %interp_method = 'linear';   % Matlab's default (leads to high frequency artifacts)
    interp_method = 'pchip';    % Piecewise cubic interpolation
    for wheel = 1:Veh(veh_num).Wheels.num
        Calc.Veh(veh_num).x_path(wheel,:) = ...
            Calc.Position.x(Calc.Time.t_0_ind:Calc.Time.t_end_ind) ... 
            - Veh(veh_num).Ax_dist(wheel) - Veh(veh_num).First_wheel_dist;
        Calc.Veh(veh_num).h_path(wheel,:) = ...
            interp1(Calc.Profile.x,Calc.Profile.h,...
            Calc.Veh(veh_num).x_path(wheel,:),interp_method);
    end % for wheel = 1:Veh(veh_num).Wheels.num

    % Removing NaN values
    Calc.Veh(veh_num).h_path(Calc.Veh(veh_num).x_path<Calc.Profile.x(1)) = ...
        Calc.Profile.h(1);
    Calc.Veh(veh_num).h_path(Calc.Veh(veh_num).x_path>Calc.Profile.x(end)) = ...
        Calc.Profile.h(end);

    % 1st derivative in time 
    Calc.Veh(veh_num).hd_path = ...
        diff(Calc.Veh(veh_num).h_path,1,2)/Calc.Solver.dt;
    Calc.Veh(veh_num).hd_path = ...
        [Calc.Veh(veh_num).hd_path(:,1),Calc.Veh(veh_num).hd_path];

    % 2nd derivative in time
    Calc.Veh(veh_num).hdd_path = ...
        diff(Calc.Veh(veh_num).hd_path,1,2)/Calc.Solver.dt;
    Calc.Veh(veh_num).hdd_path = ...
        [Calc.Veh(veh_num).hdd_path(:,1),Calc.Veh(veh_num).hdd_path];

    % First point of profile for of each wheel at level zero
    Calc.Veh(veh_num).h_path = Calc.Veh(veh_num).h_path - ...
        Calc.Veh(veh_num).h_path(:,1)*ones(1,size(Calc.Veh(veh_num).h_path,2));

    % % Graphical Check
    % figure; subplot(3,1,1); hold on; box on;
    %     plot(Calc.Veh(veh_num).x_path',Calc.Veh(veh_num).h_path'); ylabel('Profile');
    %     plot(Calc.Veh(veh_num).x_path(:,[1,end])',Calc.Veh(veh_num).h_path(:,[1,end])','.','MarkerSize',20);
    % subplot(3,1,2); hold on; box on;
    %     plot(Calc.Veh(veh_num).x_path',Calc.Veh(veh_num).hd_path'); ylabel('1st Derivative');
    %     plot(Calc.Veh(veh_num).x_path(:,[1,end])',Calc.Veh(veh_num).hd_path(:,[1,end])','.','MarkerSize',20);
    % subplot(3,1,3); hold on; box on;
    %     plot(Calc.Veh(veh_num).x_path',Calc.Veh(veh_num).hdd_path'); ylabel('2nd Derivative');
    %     plot(Calc.Veh(veh_num).x_path(:,[1,end])',Calc.Veh(veh_num).hdd_path(:,[1,end])','.','MarkerSize',20);

end % for veh_num = 1:Veh(1).Tnum

% ---- End of script ----
