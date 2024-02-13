function [] = B67_PlotModelVisualization(Calc,Veh,Track,Model)

% Generates a schematic representation of the model defined by the inputs.
% The figure is closed after the user presses ENTER, then TTB-2D proceeds
% with the simulation.

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
% Calc = Structure with Calc variables, including at least:
%   ... see script ...
% Veh = Indexed structure with Veh variables, including at least:
%   ... see script ...
% Track = Structure with Track variables, including at least:
%   ... see script ...
% Model = Structure with Model variables, including at least:
%   ... see script ...
% % ---- Outputs ----
% none
% -------------------------------------------------------------------------

if Calc.Plot.Model.P00_ModelVisualization == 1

    % Parameters for model layout
    y_value.rail = 2;
    y_value.sleepers = 1;
    y_value.ballast = 0;
    y_value.ballast_on_beam_txt = y_value.ballast + 0.2;
    y_value.beam = -0.2*Track.BallastOnBeam.included;
    y_value.wheels = y_value.rail+0.1;
    y_value.train_bottom = y_value.wheels+0.1;
    y_value.train_top = 3;
    color.rail = [0.85,0.33,0.10];
    color.sleepers = [1,1,1]*0.5;
    color.ballast = [0.79,0.78,0.51];
    color.beam = [1,1,1]*0.1;

    % Generating figure
    fig = figure; 
        set(gcf,'Name','Model visualization','NumberTitle','off');
        hold on; box on;
        % Rail
        plot(Model.Mesh.XLoc.rail_vert([1,end]),y_value.rail*[1,1],...
            'Color',color.rail,'LineWidth',2);
        % Sleepers
        plot(Model.Mesh.XLoc.sleepers,y_value.sleepers*ones(size(Model.Mesh.XLoc.sleepers)),'.',...
            'Marker','Square','MarkerSize',6,'MarkerFaceColor',color.sleepers,'MarkerEdgeColor',[1,1,1]*.2);
        % Ballast
        plot(Model.Mesh.XLoc.ballast_app,y_value.ballast*ones(size(Model.Mesh.XLoc.ballast_app)),'.',...
            'Marker','Square','MarkerSize',6,'MarkerFaceColor',color.ballast,'MarkerEdgeColor',[1,1,1]*0.2);
        plot(Model.Mesh.XLoc.ballast_aft,y_value.ballast*ones(size(Model.Mesh.XLoc.ballast_aft)),'.',...
            'Marker','Square','MarkerSize',6,'MarkerFaceColor',color.ballast,'MarkerEdgeColor',[1,1,1]*0.2);
        % Ballast on beam
        if Track.BallastOnBeam.included == 1
            plot(Model.Mesh.XLoc.beam_vert([1,end]),y_value.ballast*[1,1],'Color',color.ballast,'LineWidth',8);
        end % if Track.BallastOnBeam.included == 1
        % Beam
        plot(Model.Mesh.XLoc.beam_vert([1,end]),y_value.beam*[1,1],'Color',color.beam,'LineWidth',8);
        % Train
        Ylim = ylim;
        for veh_num = 1:Veh(1).Tnum
            plot([1;1]*Calc.Veh(veh_num).x_path(:,1)',Ylim'*ones(1,Veh(veh_num).Wheels.num),'k:');
            plot(Calc.Veh(veh_num).x_path(:,1)',zeros(1,Veh(veh_num).Wheels.num)+y_value.wheels,'ko','MarkerSize',12,'MarkerFaceColor',[1,1,1]*0.2);
            plot(Calc.Veh(veh_num).x_path([1,end],1)',[0,0]+y_value.train_bottom,'k-','LineWidth',2);
            plot(Calc.Veh(veh_num).x_path([1,end],1)',[0,0]+y_value.train_top,'k-','LineWidth',2);
            plot(Calc.Veh(veh_num).x_path(1,1)*[1,1],[y_value.train_bottom,y_value.train_top],'k-','LineWidth',2);
            plot(Calc.Veh(veh_num).x_path(end,1)*[1,1],[y_value.train_bottom,y_value.train_top],'k-','LineWidth',2);
        end % for veh_num
        % Axis limits
        ylim([y_value.beam-1,y_value.train_top+1]);
        xlim([min([Calc.Veh(Veh(1).Tnum).x_path(end,1),Model.Mesh.XLoc.rail_vert(1)]),Model.Mesh.XLoc.rail_vert(end)]);
        % Bridge limits
        plot(Model.Mesh.XLoc.beam_vert(1)*[1,1],ylim,'k--');
        plot(Model.Mesh.XLoc.beam_vert(end)*[1,1],ylim,'k--');
        % Annotations
        text(diff(Model.Mesh.XLoc.rail_vert([1,end]))*0.5,y_value.rail,'Rail',...
            'HorizontalAlignment','center','EdgeColor',color.rail,'BackgroundColor',[1,1,1],'FontSize',16,'LineWidth',2);
        text(diff(Model.Mesh.XLoc.rail_vert([1,end]))*0.5,y_value.sleepers,'Sleepers',...
            'HorizontalAlignment','center','EdgeColor',color.sleepers,'BackgroundColor',[1,1,1],'FontSize',16,'LineWidth',2);
        text(diff(Model.Mesh.XLoc.ballast_app([1,end]))*0.5,y_value.ballast,'Ballast',...
            'HorizontalAlignment','center','EdgeColor',color.ballast,'BackgroundColor',[1,1,1],'FontSize',16,'LineWidth',2);
        text(Model.Mesh.XLoc.ballast_aft(1)+diff(Model.Mesh.XLoc.ballast_aft([1,end]))*0.5,y_value.ballast,'Ballast',...
            'HorizontalAlignment','center','EdgeColor',color.ballast,'BackgroundColor',[1,1,1],'FontSize',16,'LineWidth',2);
        if Track.BallastOnBeam.included == 1
            text(Model.Mesh.XLoc.beam_vert(1)+diff(Model.Mesh.XLoc.beam_vert([1,end]))*0.75,y_value.ballast_on_beam_txt,'Ballast on beam',...
                'HorizontalAlignment','center','EdgeColor',color.ballast,'BackgroundColor',[1,1,1],'FontSize',16,'LineWidth',2);
        end % if Track.BallastOnBeam.included == 1
        text(Model.Mesh.XLoc.beam_vert(1)+diff(Model.Mesh.XLoc.beam_vert([1,end]))*0.5,y_value.beam,'Beam',...
            'HorizontalAlignment','center','EdgeColor',color.beam,'BackgroundColor',[1,1,1],'FontSize',16,'LineWidth',2);
        for veh_num = 1:Veh(1).Tnum
            text(mean(Calc.Veh(veh_num).x_path([1,end],1)),mean([y_value.train_bottom,y_value.train_top]),['Veh. ',num2str(veh_num)],...
                'HorizontalAlignment','center','FontSize',16);
        end % for veh_num = 1:Veh(1).Tnum
        % Labels and title
        xlabel('Distance');
        set(gca,'YTick',[]);
        title('Model visualization (Press ENTER to proceed with the simulation)');
        % Plot size
        aux1 = get(0,'ScreenSize');
        set(gcf,'Position',aux1)
        set(gcf,'Position',aux1+[round(aux1(3)/12),round(aux1(4)*0.7/2),-aux1(3)/6,-aux1(4)*0.7])
        set(gca,'InnerPosition',get(gca,'InnerPosition').*[0,1,0,1]+[0.05,0,0.90,0]);

    % Command window prompt
    commandwindow;
    disp('The figure "Model visualization" shows the model to simulate.')
    input('Is this correct? (Press ENTER to proceed with the simulation)');
    close(fig);
    
end % if Calc.Plot.Model.P00_ModelVisualization == 1

% ---- End of script ----
