function [Sol] = B17_CalcUat(Sol,Track,Calc,Veh)

% Function to calculate the vertical displacement of the model under the
%   wheels of each vehicle in the Train.

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
% Sol = Structure with Solutions's variables including at least:
%   .Beam.U = Nodal displacements of the beam FEM
% Track = Structure with Beam's variables, including at least:
%   ... see script ...
% Calc = Structure with Calculation variables, including at least:
%   ... see script ...
% Veh = Structure with Vehicles variables, including at least:
%   ... see script ...
% ---- Outputs ----
% Sol = Addition of fields to structure Sol:
%   .def_under = Beam deformation under the vehicle
%   .vel_under = Beam 1st derivative (in time) of deformation under the vehicle
%   .def_under_p = Beam deformation under the vehicle, with the first derivative (in space) of the shape function
%   .def_under_pp = Beam deformation under the vehicle, with the second derivative (in space) of the shape function
%   .vel_under_p = Beam 1st derivative of deformation under the vehicle, with the first derivative (in space) of the shape function
% -------------------------------------------------------------------------

% Auxiliary and local variables
Track.Rail.Mesh.Ele.a2 = [0,Track.Rail.Mesh.Ele.a];
Track.Rail.Mesh.Nodes.acum2 = [0,Track.Rail.Mesh.Nodes.acum];

% Vehicle number loop
for veh_num = 1:Veh(1).Tnum

    % Initialize
    Sol.Veh(veh_num).def_under = zeros(Veh(veh_num).Wheels.num,Calc.Solver.num_t);
    Sol.Veh(veh_num).vel_under = Sol.Veh(veh_num).def_under;
    Sol.Veh(veh_num).acc_under = Sol.Veh(veh_num).def_under;
    Sol.Veh(veh_num).def_under_p = Sol.Veh(veh_num).def_under;
    Sol.Veh(veh_num).def_under_pp = Sol.Veh(veh_num).def_under;
    Sol.Veh(veh_num).vel_under_p = Sol.Veh(veh_num).def_under;

    % Calculation of deformation using shape functions and nodal displacements
    for wheel = 1:Veh(veh_num).Wheels.num

        % Element to which each of x_path belongs to
        ele_num = Calc.Veh(veh_num).elexj(wheel,:);
        % Distance from each x_path to is left node
        x = Calc.Veh(veh_num).x_path(wheel,:) - Track.Rail.Mesh.Nodes.acum2(ele_num+1);
        % Element size at x
        a = Track.Rail.Mesh.Ele.a2(ele_num+1);
        % Element shape functions at x
        shape_fun_at_x = Track.Rail.Mesh.shape_fun(x,a)';
        shape_fun_at_x_p = Track.Rail.Mesh.shape_fun_p(x,a)';
        shape_fun_at_x_pp = Track.Rail.Mesh.shape_fun_pp(x,a)';
       
        for t = 1:Calc.Solver.num_t
            if ele_num(t) > 0
                aux1 = Track.Rail.Mesh.Ele.DOF(ele_num(t),:);
                shape_fun_at_x_t = shape_fun_at_x(t,:);
                shape_fun_at_x_p_t = shape_fun_at_x_p(t,:);
                U = Sol.Model.Nodal.U(aux1,t);
                V = Sol.Model.Nodal.V(aux1,t);
                Sol.Veh(veh_num).def_under(wheel,t) = shape_fun_at_x_t*U;
                Sol.Veh(veh_num).vel_under(wheel,t) = shape_fun_at_x_t*V;
                Sol.Veh(veh_num).acc_under(wheel,t) = shape_fun_at_x_t*Sol.Model.Nodal.A(aux1,t);
                Sol.Veh(veh_num).def_under_p(wheel,t) = shape_fun_at_x_p_t*U;
                Sol.Veh(veh_num).def_under_pp(wheel,t) = shape_fun_at_x_pp(t,:)*U;
                Sol.Veh(veh_num).vel_under_p(wheel,t) = shape_fun_at_x_p_t*V;
            end % if ele_num(t) > 0
        end % for t = 1:Calc.Solver.num_t

    end % for wheel = 1:Veh(veh_num).Wheels.num

end % for veh_num = 1:Veh(1).Tnum

% % -- Graphical Check--
% 
% veh_num = 1
% 
% figure; subplot(3,1,1);
%     plot(Calc.Solver.t,Sol.Veh(veh_num).def_under'); axis tight;
%     xlabel('Solver time (s)'); ylabel('Deformation (m)');
%     title(['Deformation under wheels of vehicle ',num2str(veh_num)]);
% subplot(3,1,2);
%     plot(Calc.Solver.t,Sol.Veh(veh_num).vel_under'); axis tight;
%     xlabel('Solver time (s)'); ylabel('Velocity (m/s)');
%     title(['Velocity under wheels of vehicle ',num2str(veh_num)]);    
% subplot(3,1,3);
%     plot(Calc.Solver.t,Sol.Veh(veh_num).acc_under'); axis tight;
%     xlabel('Solver time (s)'); ylabel('Acceleration (m/s^2)');
%     title(['Acceleration under wheels of vehicle ',num2str(veh_num)]);
% 
% figure; subplot(3,1,1);
%     plot(Calc.Solver.t,Sol.Veh(veh_num).def_under_p'); axis tight;
%     xlabel('Solver time (s)'); ylabel('Deformation (m)');
%     title(['Deformation under wheels (1st derivative) of vehicle ',num2str(veh_num)]);
% subplot(3,1,2);
%     plot(Calc.Solver.t,Sol.Veh(veh_num).def_under_pp'); axis tight;
%     xlabel('Solver time (s)'); ylabel('Velocity (m/s)');
%     title(['Deformation under wheels (2nd derivative) of vehicle ',num2str(veh_num)]);    
% subplot(3,1,3);
%     plot(Calc.Solver.t,Sol.Veh(veh_num).vel_under_p'); axis tight;
%     xlabel('Solver time (s)'); ylabel('Acceleration (m/s^2)');
%     title(['Velocity under wheels (1st derivative) of vehicle ',num2str(veh_num)]);

% ---- End of function ----
