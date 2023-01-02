% Script with all the results plotting

% Not a function.
% Plots the requested figures specified in A04_Options

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

% ------------------------- Vehicle Results ------------------------------- 

if Calc.Plot.Veh.P01_VertDisp == 1
    % ---- Vehicles Vertical Displacements ----
    figure;
    for veh_num = 1:Train.Veh(1).Tnum
        subplot(Train.Veh(1).Tnum,1,veh_num)
        plot(Calc.Solver.t,Sol.Veh(veh_num).U(1:3,:) - ...
            Sol.Veh(veh_num).U0(1:3)*ones(1,Calc.Solver.num_t));
        axis tight;
        xlabel('Time (s)'); ylabel([{'Vertical'};{'Disp. (m)'}]);
        title(['Vehicle ',num2str(veh_num)]);
        legend('Main body','Bogie 1','Bogie 2');
    end % for veh_num = 1:Train.Veh(1).Tnum
    clear veh_num
end % if Calc.Plot.Veh.P01_VertDisp == 1

if Calc.Plot.Veh.P02_VertVel == 1
    % ---- Vehicles Vertical Velocities ----
    figure;
    for veh_num = 1:Train.Veh(1).Tnum
        subplot(Train.Veh(1).Tnum,1,veh_num)
        plot(Calc.Solver.t,Sol.Veh(veh_num).V(1:3,:)); axis tight;
        xlabel('Time (s)'); ylabel([{'Vertical'};{'Vel. (m/s)'}]);
        title(['Vehicle ',num2str(veh_num)]);
        legend('Main body','Bogie 1','Bogie 2');
    end % for veh_num = 1:Train.Veh(1).Tnum
    clear veh_num
end % if Calc.Plot.Veh.P02_VertVel == 1

if Calc.Plot.Veh.P03_VertAcc == 1
    % ---- Vehicles Vertical accelerations ----
    figure;
    for veh_num = 1:Train.Veh(1).Tnum
        subplot(Train.Veh(1).Tnum,1,veh_num)
        plot(Calc.Solver.t,Sol.Veh(veh_num).A(1:3,:)); axis tight;
        xlabel('Time (s)'); ylabel([{'Vertical'};{'Acc. (m/s^2)'}]);
        title(['Vehicle ',num2str(veh_num)]);
        legend('Main body','Bogie 1','Bogie 2');
    end % for veh_num = 1:Train.Veh(1).Tnum
    clear veh_num
end % if Calc.Plot.Veh.P03_VertAcc == 1

if Calc.Plot.Veh.P04_ContactForce_t > 0
    % ---- Contact force (In time) ----
    figure;
    for veh_num = 1:Train.Veh(1).Tnum
        subplot(Train.Veh(1).Tnum,1,veh_num)
        plot(Calc.Solver.t,Sol.Veh(veh_num).F_onBeam/1000); axis tight;
        if Calc.Plot.Veh.P04_ContactForce_t == 2
            hold on; plot(Calc.Solver.t([1,end]),[0,0],'k--'); axis tight;
            Ylim = ylim; ylim(Ylim+diff(Ylim)*0.1*[-1,1]);
        end % if Calc.Plot.Veh.P04_ContactForce_t == 2
        xlabel('Time (s)'); ylabel([{'Vertical'};{'Contact Force (kN)'}]);
        title(['Vehicle ',num2str(veh_num)]);
        legend('Wheel 1','Wheel 2','Wheel 3','Wheel 4');
    end % for veh_num = 1:Train.Veh(1).Tnum
    clear veh_num
end % if Calc.Plot.Veh.P04_ContactForce_t > 0

if Calc.Plot.Veh.P05_ContactForce_x > 0
    % ---- Contact force (In space) ----
    figure;
    for veh_num = 1:Train.Veh(1).Tnum
        subplot(Train.Veh(1).Tnum,1,veh_num); hold on; box on;
        for wheel = 1:Train.Veh(veh_num).Wheels.num
            plot(Calc.Veh(veh_num).x_path(wheel,:),Sol.Veh(veh_num).F_onBeam(wheel,:)/1000); axis tight;
            xlim([Calc.Veh(end).x_path(end,1),Calc.Position.x(end)]);
        end % for wheel = 1:Train.Veh(veh_num).Wheels.num
        if Calc.Plot.Veh.P05_ContactForce_x == 2
            plot(Calc.Position.x([1,end]),[0,0],'k--'); axis tight;
            Ylim = ylim; ylim(Ylim+diff(Ylim)*0.1*[-1,1]);
        end % if Calc.Plot.Veh.P05_ContactForce_x == 2
        xlabel('Distance (m)'); ylabel([{'Vertical'};{'Contact Force (kN)'}]);
        title(['Vehicle ',num2str(veh_num)]);
        legend('Wheel 1','Wheel 2','Wheel 3','Wheel 4');
    end % for veh_num = 1:Train.Veh(1).Tnum
    clear veh_num
end % if Calc.Plot.Veh.P05_ContactForce_x > 0

% -------------------------- Model Results --------------------------------

if Calc.Plot.Model.P01_ModelDef >= 0
    % ---- Model deformation at one particular time instant ----
    t_per = Calc.Plot.Model.P01_ModelDef;
    ylim_add_per = 10;              % Margins addition on Y axis in percentage [%]
    t = round(Calc.Solver.num_t*t_per/100)+1-ceil(t_per/100);
    figure; subplot(3,1,1); hold on; box on;
        plot(Model.Mesh.XLoc.rail_vert,Sol.Model.Nodal.U(Model.Mesh.DOF.rail_vert,t)*1000,'b');
        axis tight; Ylim = ylim; Ylim = Ylim + sign(Ylim).*diff(Ylim)*ylim_add_per/100/2; ylim(Ylim);
        for veh_num = 1:Train.Veh(1).Tnum
            plot([1;1]*Calc.Veh(veh_num).x_path(:,t)',Ylim'*ones(1,Train.Veh(veh_num).Wheels.num),'k:');
        end % for veh_num
        ylabel([{'Rail'},{'Vert. Disp (mm)'}]);
        title(['Model vertical displacements for t = ',num2str(round(Calc.Solver.t(t),2)),'s']);
    subplot(3,1,2); hold on; box on;
        plot(Model.Mesh.XLoc.sleepers,Sol.Model.Nodal.U(Model.Mesh.DOF.sleepers,t)*1000,'b.');
        axis tight; Ylim = ylim; Ylim = Ylim + sign(Ylim).*diff(Ylim)*ylim_add_per/100/2; ylim(Ylim);
        for veh_num = 1:Train.Veh(1).Tnum
            plot([1;1]*Calc.Veh(veh_num).x_path(:,t)',Ylim'*ones(1,Train.Veh(veh_num).Wheels.num),'k:');
        end % for veh_num
        ylabel([{'Sleepers'},{'Vert. Disp (mm)'}]);
    subplot(3,1,3); hold on; box on;
        plot(Model.Mesh.XLoc.ballast_app,Sol.Model.Nodal.U(Model.Mesh.DOF.ballast_app,t)*1000,'b.');
        plot(Model.Mesh.XLoc.beam_vert,Sol.Model.Nodal.U(Model.Mesh.DOF.beam_vert,t)*1000,'b');
        plot(Model.Mesh.XLoc.ballast_aft,Sol.Model.Nodal.U(Model.Mesh.DOF.ballast_aft,t)*1000,'b.');
        axis tight; Ylim = ylim; Ylim = Ylim + sign(Ylim).*diff(Ylim)*ylim_add_per/100/2; ylim(Ylim);
        for veh_num = 1:Train.Veh(1).Tnum
            plot([1;1]*Calc.Veh(veh_num).x_path(:,t)',Ylim'*ones(1,Train.Veh(veh_num).Wheels.num),'k:');
        end % for veh_num
        ylabel([{'Ballast and Beam'},{'Vert. Disp (mm)'}]);
        xlabel('Distance (m)');
    clear t t_per veh_num xdata ylim_add_per Ylim
end % if Calc.Plot.Model.P01_ModelDef >=0

if Calc.Plot.Model.P02_ModelRot >= 0
    % ---- Model rotations at one particular time instant ----
    t_per = Calc.Plot.Model.P02_ModelRot;
    ylim_add_per = 10;              % Margins addition on Y axis in percentage [%]
    t = round(Calc.Solver.num_t*t_per/100)+1-ceil(t_per/100);
    figure; subplot(3,1,1); hold on; box on;
        plot(Model.Mesh.XLoc.rail_vert,Sol.Model.Nodal.U(Model.Mesh.DOF.rail_vert+1,t),'b');
        axis tight; Ylim = ylim; Ylim = Ylim + sign(Ylim).*diff(Ylim)*ylim_add_per/100/2; ylim(Ylim);
        for veh_num = 1:Train.Veh(1).Tnum
            plot([1;1]*Calc.Veh(veh_num).x_path(:,t)',Ylim'*ones(1,Train.Veh(veh_num).Wheels.num),'k:');
        end % for veh_num
        ylabel([{'Rail'},{'Rotation (rad)'}]);
        title(['Model rotations for t = ',num2str(round(Calc.Solver.t(t),2)),'s']);
    subplot(3,1,3); hold on; box on;
        plot(Model.Mesh.XLoc.beam_vert,Sol.Model.Nodal.U(Model.Mesh.DOF.beam_vert+1,t),'b');
        axis tight; Ylim = ylim; Ylim = Ylim + sign(Ylim).*diff(Ylim)*ylim_add_per/100/2; ylim(Ylim);
        for veh_num = 1:Train.Veh(1).Tnum
            plot([1;1]*Calc.Veh(veh_num).x_path(:,t)',Ylim'*ones(1,Train.Veh(veh_num).Wheels.num),'k:');
        end % for veh_num
        xlim(Track.Rail.Mesh.Nodes.acum([1,end]))
        ylabel([{'Beam'},{'Rotation (rad)'}]);
        xlabel('Distance (m)');
    clear t t_per veh_num xdata ylim_add_per Ylim
end % if Calc.Plot.Model.P02_ModelRot >=0

% --------------------------- Beam Results --------------------------------

if Calc.Plot.Beam.P01_DispContour == 1
    % ---- Beam displacements ----
    figure; 
        [~,h] = contourf(Beam.Mesh.Nodes.acum,Calc.Solver.t,Sol.Beam.U.xt'*1000,20);
        set(h,'EdgeColor','none'); colorbar;
        cmap = colormap; colormap(flipud(cmap));
        for veh_num = 1:Train.Veh(1).Tnum
            hold on; plot(Calc.Veh(veh_num).x_path-Calc.Profile.L_Aw,Calc.Solver.t,'k--');
        end % for veh_num = 1:Train.Veh(1).Tnum
        xlim(Beam.Mesh.Nodes.acum([1,end]));
        xlabel('Distance (m)'); ylabel('Time (s)');
        title('Beam Vertical Displacements (mm)');
        clear h cmap veh_num
end % if Calc.Plot.Beam.P01_DispContour == 1
    
if Calc.Plot.Beam.P02_StaticDispContour == 1
    % ---- Beam Static displacements ----
    figure; 
        [~,h] = contourf(Beam.Mesh.Nodes.acum,Calc.Solver.t,Sol.Beam.StaticU.xt'*1000,20);
        set(h,'EdgeColor','none'); colorbar;
        cmap = colormap; colormap(flipud(cmap));
        for veh_num = 1:Train.Veh(1).Tnum
            hold on; plot(Calc.Veh(veh_num).x_path-Calc.Profile.L_Aw,Calc.Solver.t,'k--')
        end % for veh_num = 1:Train.Veh(1).Tnum
        xlim(Beam.Mesh.Nodes.acum([1,end]));
        xlabel('Distance (m)'); ylabel('Time (s)');
        title('Beam Static Vertical Displacements (mm)');
        clear h cmap veh_num
end % if Calc.Plot.Beam.P02_StaticDispContour == 1

if Calc.Plot.Beam.P03_BMContour == 1
    % ---- Beam Bending Moments ----
    figure; 
        [~,h] = contourf(Beam.Mesh.Nodes.acum,Calc.Solver.t,Sol.Beam.BM.xt'/1000,20);
        set(h,'EdgeColor','none'); colorbar;
        for veh_num = 1:Train.Veh(1).Tnum
            hold on; plot(Calc.Veh(veh_num).x_path-Calc.Profile.L_Aw,Calc.Solver.t,'k--')
        end % for veh_num = 1:Train.Veh(1).Tnum
        xlim(Beam.Mesh.Nodes.acum([1,end]));
        xlabel('Distance (m)'); ylabel('Time (s)');
        title('Beam Bending Moments (kN*m)');
        clear h veh_num
end % if Calc.Plot.Beam.P03_BMContour == 1

if Calc.Plot.Beam.P04_StaticBMContour == 1
    % ---- Beam Static Bending Moments ----
    figure; 
        [~,h] = contourf(Beam.Mesh.Nodes.acum,Calc.Solver.t,Sol.Beam.StaticBM.xt'/1000,20);
        set(h,'EdgeColor','none'); colorbar;
        for veh_num = 1:Train.Veh(1).Tnum
            hold on; plot(Calc.Veh(veh_num).x_path-Calc.Profile.L_Aw,Calc.Solver.t,'k--')
        end % for veh_num = 1:Train.Veh(1).Tnum
        xlim(Beam.Mesh.Nodes.acum([1,end]));
        xlabel('Distance (m)'); ylabel('Time (s)');
        title('Beam Static Bending Moments (kN*m)');
        clear h veh_num
end % if Calc.Plot.Beam.P04_StaticBMContour == 1

if Calc.Plot.Beam.P05_ShearContour == 1
    % ---- Beam Shear ----
    figure; 
        [~,h] = contourf(Beam.Mesh.Nodes.acum,Calc.Solver.t,Sol.Beam.Shear.xt'/1000,20);
        set(h,'EdgeColor','none'); colorbar;
        for veh_num = 1:Train.Veh(1).Tnum
            hold on; plot(Calc.Veh(veh_num).x_path-Calc.Profile.L_Aw,Calc.Solver.t,'k--')
        end % for veh_num = 1:Train.Veh(1).Tnum
        xlim(Beam.Mesh.Nodes.acum([1,end]));
        xlabel('Distance (m)'); ylabel('Time (s)');
        title('Beam Shear (kN)');
        clear h veh_num
end % if Calc.Plot.Beam.P05_ShearContour == 1

if Calc.Plot.Beam.P06_StaticShearContour == 1
    % ---- Beam Static Shear ----
    figure; 
        [~,h] = contourf(Beam.Mesh.Nodes.acum,Calc.Solver.t,Sol.Beam.StaticShear.xt'/1000,20);
        set(h,'EdgeColor','none'); colorbar;
        for veh_num = 1:Train.Veh(1).Tnum
            hold on; plot(Calc.Veh(veh_num).x_path-Calc.Profile.L_Aw,Calc.Solver.t,'k--')
        end % for veh_num = 1:Train.Veh(1).Tnum
        xlim(Beam.Mesh.Nodes.acum([1,end]));
        xlabel('Distance (m)'); ylabel('Time (s)');
        title('Beam Static Shear (kN)');
        clear h veh_num
end % if Calc.Plot.Beam.P06_StaticShearContour == 1

if Calc.Plot.Beam.P07_VertAccContour == 1
    % ---- Beam vertical Acceleration ----
    figure; 
        [~,h] = contourf(Beam.Mesh.Nodes.acum,Calc.Solver.t,Sol.Beam.Acc.xt',20);
        set(h,'EdgeColor','none'); colorbar;
        % % Alternative plotting, if contourf is slow
        %pcolor(Beam.Mesh.Nodes.acum,Calc.Solver.t,Sol.Beam.Acc.xt'); 
        %shading interp; colorbar;
        for veh_num = 1:Train.Veh(1).Tnum
            hold on; plot(Calc.Veh(veh_num).x_path-Calc.Profile.L_Aw,Calc.Solver.t,'k--')
        end % for veh_num = 1:Train.Veh(1).Tnum
        xlim(Beam.Mesh.Nodes.acum([1,end]));
        xlabel('Distance (m)'); ylabel('Time (s)');
        title('Beam Vertical Acceleration (m/s^2)');
        clear h
end % if Calc.Plot.Beam.P07_VertAccContour == 1

if Calc.Options.num_calc_beam_sections > 0

    if Calc.Plot.Beam.P08_Sections_BeamVertDisp == 1
        % ---- Beam vertical displacement at selected sections ----
        figure; 
            for section = 1:Calc.Options.num_calc_beam_sections
                subplot(Calc.Options.num_calc_beam_sections,1,section)
                plot(Calc.Solver.t,Sol.Beam.U.sections_t(section,:)*1000); hold on;
                plot(Calc.Solver.t,Sol.Beam.StaticU.sections_t(section,:)*1000,'--');        
                xlabel('Time (s)');
                ylabel([{['Section ',num2str(round(Calc.Options.calc_beam_sections(section),2)),'m'];...
                    'Displacements';'(mm)'}]);
                axis tight;
                if section == Calc.Options.num_calc_beam_sections
                    legend('Total','Static');
                end % if section == Calc.Options.num_calc_beam_sections
            end % for section = 1:Calc.Options.num_calc_beam_sections
    end % if Calc.Plot.Beam.P08_Sections_BeamVertDisp == 1

    if Calc.Plot.Beam.P09_Sections_BeamBM == 1
        % ---- Beam Bending Moment at selected sections ----
        figure; 
            for section = 1:Calc.Options.num_calc_beam_sections
                subplot(Calc.Options.num_calc_beam_sections,1,section)
                plot(Calc.Solver.t,Sol.Beam.BM.sections_t(section,:)/1000); hold on;
                plot(Calc.Solver.t,Sol.Beam.StaticBM.sections_t(section,:)/1000,'--');        
                xlabel('Time (s)');
                ylabel([{['Section ',num2str(round(Calc.Options.calc_beam_sections(section),2)),'m'];...
                    'Bending Moment';'(kN/m)'}]);
                axis tight;
                if section == Calc.Options.num_calc_beam_sections
                    legend('Total','Static');
                end % if section == Calc.Options.num_calc_beam_sections
            end % for section = 1:Calc.Options.num_calc_beam_sections
    end % if Calc.Plot.Beam.P09_Sections_BeamBM == 1

    if Calc.Plot.Beam.P10_Sections_BeamShear == 1
        % ---- Beam Shear at selected sections ----
        figure; 
            for section = 1:Calc.Options.num_calc_beam_sections
                subplot(Calc.Options.num_calc_beam_sections,1,section)
                plot(Calc.Solver.t,Sol.Beam.Shear.sections_t(section,:)/1000); hold on;
                plot(Calc.Solver.t,Sol.Beam.StaticShear.sections_t(section,:)/1000,'--');        
                xlabel('Time (s)');
                ylabel([{['Section ',num2str(round(Calc.Options.calc_beam_sections(section),2)),'m'];...
                    'Shear';'(kN)'}]);
                axis tight;
                if section == Calc.Options.num_calc_beam_sections
                    legend('Total','Static','Location','Best');
                end % if section == Calc.Options.num_calc_beam_sections
            end % for section = 1:Calc.Options.num_calc_beam_sections
    end % if Calc.Plot.Beam.P10_Sections_BeamShear == 1

    if Calc.Plot.Beam.P11_Sections_BeamAcc == 1
        % ---- Beam Acceleration at selected sections ----
        figure; 
            for section = 1:Calc.Options.num_calc_beam_sections
                subplot(Calc.Options.num_calc_beam_sections,1,section)
                plot(Calc.Solver.t,Sol.Beam.Acc.sections_t(section,:));
                xlabel('Time (s)');
                ylabel([{['Section ',num2str(round(Calc.Options.calc_beam_sections(section),2)),'m'];...
                    'Acceleration';'(m/s^2)'}]);
                axis tight;
            end % for section = 1:Calc.Options.num_calc_beam_sections
            clear section
    end % if Calc.Plot.Beam.P11_Sections_BeamAcc == 1

end % if Calc.Options.num_calc_beam_sections > 0

% ---- End of script ----
