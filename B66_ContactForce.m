function [Sol] = B66_ContactForce(Sol,Track,Calc,Train)

% Calculation of vertical contact forces for each vehicle
% Note: Contact force calculation looks good only if Calc.Options.redux = 0
%   When the redux model option is ised, then the vehicle starts the simulation
%   on a flat rigid surface. At the simulation progresses, each wheel enter
%   the track and produces large vertical deformations on the track, which
%   at the same time produces large oscilations in the contact force.

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
% Veh = Indexed structure with Veh variables, including at least:
%   ... see script ...
% Model = Structure with Model variables, including at least:
%   ... see script ...
% Calc = Structure with Calc variables, including at least:
%   ... see script ...
% Track = Structure with Track variables, including at least:
%   ... see script ...
% Sol = Structure with Sol variables, including at least:
%   ... see script ...
% % ---- Outputs ----
% Sol = Additional fields to Sol structure variable
% ... see script ...
% -------------------------------------------------------------------------

% Auxiliary variables
ones_1_x_num_t = ones(1,Calc.Solver.num_t);

% **** VBI ****
if Calc.Options.VBI == 1
    
    % Deformation under each wheel
    [Sol] = B17_CalcUat(Sol,Track,Calc,Train.Veh);

    % Force on Beam (Contact Force)
    for veh_num = 1:Train.Veh(1).Tnum

        ind = (Calc.Veh(veh_num).x_path>=0);
        
        Sol.Veh(veh_num).F_onBeam = ...
            ((Train.Veh(veh_num).Wheels.N2w * Sol.Veh(veh_num).U) ...
                - Sol.Veh(veh_num).def_under - Calc.Veh(veh_num).h_path.*ind) ...
                .* (Train.Veh(veh_num).Susp.Prim.k' * ones_1_x_num_t) ...
            + ((Train.Veh(veh_num).Wheels.N2w * Sol.Veh(veh_num).V) ...
                - Sol.Veh(veh_num).vel_under - Sol.Veh(veh_num).def_under_p*Train.vel...
                - Calc.Veh(veh_num).hd_path.*ind) ...
                .* (Train.Veh(veh_num).Susp.Prim.c' * ones_1_x_num_t) ... 
            - (Sol.Veh(veh_num).acc_under + Sol.Veh(veh_num).def_under_pp*Train.vel^2 ...
                + 2*Sol.Veh(veh_num).vel_under_p*Train.vel) ...
                .* (Train.Veh(veh_num).Wheels.m' * ones_1_x_num_t) ...
            + (Train.Veh(veh_num).Wheels.m' * ones_1_x_num_t) .* Calc.Cte.grav;

        Sol.Veh(veh_num).F_onBeam_max = max(Sol.Veh(veh_num).F_onBeam(:));
        Sol.Veh(veh_num).F_onBeam_min = min(Sol.Veh(veh_num).F_onBeam(:));

        % Checking contact (while on the bridge)
        L1 = Calc.Profile.L_Approach + Calc.Position.x_0*Calc.Options.redux_factor;
        L2 = Calc.Profile.L_Approach + Calc.Profile.L_bridge + Calc.Position.x_0*Calc.Options.redux_factor;
        ind = and(Calc.Veh(veh_num).x_path>=L1,Calc.Veh(veh_num).x_path<=L2);
        Sol.Veh(veh_num).F_onBridge_max = max(max(Sol.Veh(veh_num).F_onBeam(ind)));
        Sol.Veh(veh_num).F_onBridge_min = min(min(Sol.Veh(veh_num).F_onBeam(ind)));
        Sol.Veh(veh_num).contactLost = Sol.Veh(veh_num).F_onBridge_max>0;
        
%         %-- Graphical Check --
%         if veh_num == 1
%             figure;
%         end % if veh_num == 1
%         subplot(ceil(Train.Veh(1).Tnum/2),2,veh_num);
%             hold on; box on;
%             plot(Calc.Veh(veh_num).x_path',Sol.Veh(veh_num).F_onBeam');
%             axis tight;
%             plot(L1*[1,1],ylim,'k--');
%             plot(L2*[1,1],ylim,'k--');
%             if Sol.Veh(veh_num).F_onBridge_max > 0 
%                 plot(xlim,[0,0],'r--');
%             end % if Sol.Veh(veh_num).F_onBridge_max > 0                 
%             plot([L1,L2],[1,1]*Sol.Veh(veh_num).F_onBridge_max,'k-');
%             plot([L1,L2],[1,1]*Sol.Veh(veh_num).F_onBridge_min,'k-');
%             xlim([L1,L2]);
%             ylim([Sol.Veh(veh_num).F_onBridge_min,Sol.Veh(veh_num).F_onBridge_max]);
%             plot(xlim,[1,1]*Train.Veh(veh_num).sta_loads(1),'r--'); %axis tight;
%             title(['Vehicle ',num2str(veh_num),' (',num2str(Sol.Veh(veh_num).contactLost),')']);

    end % for veh_num = 1:Train.Veh(1).Tnum

% **** Moving Force ****
elseif Calc.Options.VBI == 0

    % Force on Beam (Contact Force)
    for veh_num = 1:Train.Veh(1).Tnum

        Sol.Veh(veh_num).F_onBeam = ...
            ((Train.Veh(veh_num).Wheels.N2w * Sol.Veh(veh_num).U)) ...
                .* (Train.Veh(veh_num).Susp.Prim.k' * ones_1_x_num_t) ...
            + ((Train.Veh(veh_num).Wheels.N2w * Sol.Veh(veh_num).V)) ...
                .* (Train.Veh(veh_num).Susp.Prim.c' * ones_1_x_num_t) ...
            + (Train.Veh(veh_num).Wheels.m' * ones_1_x_num_t) .* Calc.Cte.grav;
        
        Sol.Veh(veh_num).F_onBeam_max = max(Sol.Veh(veh_num).F_onBeam(:));
        Sol.Veh(veh_num).F_onBeam_min = min(Sol.Veh(veh_num).F_onBeam(:));

        % Checking contact (while on the bridge)
        L1 = Calc.Profile.L_Approach;
        L2 = Calc.Profile.L_Approach + Calc.Profile.L_bridge;
        ind = and(Calc.Veh(veh_num).x_path>=L1,Calc.Veh(veh_num).x_path<=L2);
        Sol.Veh(veh_num).contactLost = max(max(Sol.Veh(veh_num).F_onBeam.*ind))>0;

    end % for veh_num = 1:Train.Veh(1).Tnum

end % if Calc.Options.VBI == 1

% ---- Checking contact ----
if max([Sol.Veh(:).contactLost]) > 0
    Sol.contactLost = 1;
    disp('There is no permanent contact between wheels and rail');
else
    Sol.contactLost = 0;
end % if max([Sol.Veh(:).contactLost]) > 0

% % -- Graphical check of components --
% veh_num = 1;
% [Sol] = B17_CalcUat(Sol,Track,Calc,Train.Veh);
% figure;        
% subplot(3,1,1); plot(((Train.Veh(veh_num).Wheels.N2w * Sol.Veh(veh_num).U).* (Train.Veh(veh_num).Susp.Prim.k' * ones_1_x_num_t))'); axis tight;
% subplot(3,1,2); plot((-Sol.Veh(veh_num).def_under.* (Train.Veh(veh_num).Susp.Prim.k' * ones_1_x_num_t))'); axis tight;
% subplot(3,1,3); plot(((- Calc.Veh(veh_num).h_path).* (Train.Veh(veh_num).Susp.Prim.k' * ones_1_x_num_t))'); axis tight;
% figure;
% subplot(4,1,1); plot(((Train.Veh(veh_num).Wheels.N2w * Sol.Veh(veh_num).V).* (Train.Veh(veh_num).Susp.Prim.c' * ones_1_x_num_t))'); axis tight;
% subplot(4,1,2); plot((- Sol.Veh(veh_num).vel_under .* (Train.Veh(veh_num).Susp.Prim.c' * ones_1_x_num_t))'); axis tight;
% subplot(4,1,3); plot((- Sol.Veh(veh_num).def_under_p*Train.vel .* (Train.Veh(veh_num).Susp.Prim.c' * ones_1_x_num_t))'); axis tight;
% subplot(4,1,4); plot(((- Calc.Veh(veh_num).hd_path) .* (Train.Veh(veh_num).Susp.Prim.c' * ones_1_x_num_t))'); axis tight;
% figure;
% subplot(4,1,1); plot((-Sol.Veh(veh_num).acc_under.*(Train.Veh(veh_num).Wheels.m' * ones_1_x_num_t))'); axis tight;
% subplot(4,1,2); plot((-Sol.Veh(veh_num).def_under_pp*Train.vel^2.*(Train.Veh(veh_num).Wheels.m' * ones_1_x_num_t))'); axis tight;
% subplot(4,1,3); plot(((-2*Sol.Veh(veh_num).vel_under_p*Train.vel).*(Train.Veh(veh_num).Wheels.m' * ones_1_x_num_t))'); axis tight;
% subplot(4,1,4); plot(((Train.Veh(veh_num).Wheels.m' * ones_1_x_num_t) .* Calc.Cte.grav)'); axis tight;

% ---- End of script ----
