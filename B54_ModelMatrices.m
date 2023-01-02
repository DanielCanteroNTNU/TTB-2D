function [Model] = B54_ModelMatrices(Beam,Track,Calc)

% Assembles the coupled model (Track+Beam) system matrices

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
% Beam = Structure with Beam variables, including at least:
%   ... see script ...
% Track = Structure with Track variables, including at least:
%   ... see script ...
% Calc = Structure with Calculation variables, including at least:
%   ... see script ...
% % ---- Outputs ----
% Model = New structure variable with the system matrices for 
%   the Track and Beam models coupled together
%   ... see script ...
% -------------------------------------------------------------------------

% ---- Counting ----
% Total
Track.Sleeper.Tnum = round(Calc.Profile.L/Track.Sleeper.spacing) + 1;
% Approach
Track.Sleeper.num_app = round((Calc.Profile.max_TL*Calc.Options.redux_factor+ ...
    Calc.Profile.L_Approach)/Track.Sleeper.spacing);
if Calc.Profile.extra_L < Calc.Cte.tol
    Track.Sleeper.num_onbeam = round(Beam.Prop.L/Track.Sleeper.spacing) + 1;
    Track.Sleeper.num_aft = round((Calc.Profile.L - ...
        (Calc.Profile.max_TL*Calc.Options.redux_factor+Calc.Profile.L_Approach+ ...
        Beam.Prop.L+Calc.Profile.extra_L))/Track.Sleeper.spacing);
else
    Track.Sleeper.num_onbeam = floor(Beam.Prop.L/Track.Sleeper.spacing) + 1;
    Track.Sleeper.num_aft = round((Calc.Profile.L - ...
        (Calc.Profile.max_TL*Calc.Options.redux_factor+Calc.Profile.L_Approach+...
        Beam.Prop.L+Calc.Profile.extra_L))/Track.Sleeper.spacing) + 1;
end % if Calc.Profile.extra_L < Calc.Cte.tol

% % Check (Should be zero)
% Track.Sleeper.Tnum - (Track.Sleeper.num_app + Track.Sleeper.num_onbeam + Track.Sleeper.num_aft)

% ---- DOF indices ----
Model.Mesh.DOF.rail = 1:Track.Rail.Mesh.DOF.Tnum;
Model.Mesh.DOF.rail_vert = Model.Mesh.DOF.rail(1:2:end);
Model.Mesh.DOF.rail_vert_at_sleepers = ...
    (1:Track.Rail.Mesh.Ele.num_per_spacing:Track.Rail.Mesh.Nodes.Tnum)*2-1;
Model.Mesh.DOF.sleepers = (1:Track.Sleeper.Tnum) + Track.Rail.Mesh.DOF.Tnum;
Model.Mesh.DOF.sleepers_app = Model.Mesh.DOF.sleepers(1:Track.Sleeper.num_app);
Model.Mesh.DOF.sleepers_onbeam = ...
    Model.Mesh.DOF.sleepers(Track.Sleeper.num_app+(1:Track.Sleeper.num_onbeam));
Model.Mesh.DOF.sleepers_aft = ...
    Model.Mesh.DOF.sleepers(Track.Sleeper.num_app+Track.Sleeper.num_onbeam + ...
    (1:Track.Sleeper.num_aft));
Model.Mesh.DOF.ballast_app = Model.Mesh.DOF.sleepers(end) + ...
    (1:Track.Sleeper.num_app);
Model.Mesh.DOF.beam = Model.Mesh.DOF.ballast_app(end) + (1:Beam.Mesh.DOF.Tnum);
Model.Mesh.DOF.beam_vert = Model.Mesh.DOF.beam(1:2:end);
Model.Mesh.DOF.beam_vert_under_sleeper = ...
    Model.Mesh.DOF.beam_vert(1:Beam.Mesh.Ele.num_per_spacing:end);
Model.Mesh.DOF.ballast_aft = Model.Mesh.DOF.beam(end) + (1:Track.Sleeper.num_aft);

if Track.Sleeper.num_aft == 0
    Model.Mesh.DOF.Tnum = Model.Mesh.DOF.beam(end);
else
    Model.Mesh.DOF.Tnum = Model.Mesh.DOF.ballast_aft(end);
end % if Track.Sleeper.num_aft == 0

% ---- X location of some DOF ---- (Useful for plotting)
Model.Mesh.XLoc.rail_vert = Track.Rail.Mesh.Nodes.acum;
Model.Mesh.XLoc.sleepers = ...
    (0:length(Model.Mesh.DOF.sleepers)-1)*Track.Sleeper.spacing;
Model.Mesh.XLoc.ballast_app = ...
    (0:length(Model.Mesh.DOF.ballast_app)-1)*Track.Sleeper.spacing;
Model.Mesh.XLoc.beam_vert = Beam.Mesh.Nodes.acum + Calc.Profile.L_Approach + ...
    Calc.Profile.max_TL*Calc.Options.redux_factor;
Model.Mesh.XLoc.ballast_aft = ...
    ((0:length(Model.Mesh.DOF.ballast_aft)-1)+1)*Track.Sleeper.spacing + ...
    Calc.Profile.L_Approach + Beam.Prop.L + ...
    Calc.Profile.max_TL*Calc.Options.redux_factor + Calc.Profile.extra_L;

% % Graphical check
% figure; hold on; box on;
% plot(Model.Mesh.XLoc.rail_vert,1.5*ones(size(Model.Mesh.XLoc.rail_vert)),'.');
% plot(Model.Mesh.XLoc.sleepers,1*ones(size(Model.Mesh.XLoc.sleepers)),'g.');
% plot(Model.Mesh.XLoc.ballast_app,0.5*ones(size(Model.Mesh.XLoc.ballast_app)),'r.');
% plot(Model.Mesh.XLoc.beam_vert,0.5*ones(size(Model.Mesh.XLoc.beam_vert)),'k.');
% aux1 = 1:Beam.Mesh.Ele.num_per_spacing:length(Model.Mesh.XLoc.beam_vert);
% plot(Model.Mesh.XLoc.beam_vert(aux1),0.5*ones(size(Model.Mesh.XLoc.beam_vert(aux1))),'ko');
% plot(Model.Mesh.XLoc.ballast_aft,0.5*ones(size(Model.Mesh.XLoc.ballast_aft)),'r.');
% ylim([0,2]);
% legend('Rail nodes','Sleepers','Ballast app.','Beam','Beam under Sleeper','Ballast aft.');

% Temporary variable name change
if Track.PadUnderSleeperOnBeam.included == 1
    Track.BallastOnBeam.Prop.m = 0;
    Track.BallastOnBeam.Prop.c = Track.PadUnderSleeperOnBeam.Prop.c;
    Track.BallastOnBeam.Prop.k = Track.PadUnderSleeperOnBeam.Prop.k;
end % if Track.PadUnderSleeperOnBeam.included == 1

% ---------------------- Building Global matrices -------------------------

% Initialize matrices
% Model.Mesh.Mg = zeros(Model.Mesh.DOF.Tnum);
% Model.Mesh.Cg = zeros(Model.Mesh.DOF.Tnum);
% Model.Mesh.Kg = zeros(Model.Mesh.DOF.Tnum);
Model.Mesh.Mg = sparse(Model.Mesh.DOF.Tnum,Model.Mesh.DOF.Tnum);
Model.Mesh.Cg = Model.Mesh.Mg;
Model.Mesh.Kg = Model.Mesh.Mg;

% ---- Diagonal Elementens ----

% Track
Model.Mesh.Mg = funAdd1(Model.Mesh.Mg,Model.Mesh.DOF.rail,Track.Rail.Mesh.Mg);
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.rail,Track.Rail.Mesh.Cg);
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.rail,Track.Rail.Mesh.Kg);

% Pads to rail DOF
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.rail_vert_at_sleepers,...
    funDiag(Track.Sleeper.Tnum,Track.Pad.Prop.c));
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.rail_vert_at_sleepers,...
    funDiag(Track.Sleeper.Tnum,Track.Pad.Prop.k));

% Pads to sleepers DOF
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.sleepers,...
    funDiag(Track.Sleeper.Tnum,Track.Pad.Prop.c));
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.sleepers,...
    funDiag(Track.Sleeper.Tnum,Track.Pad.Prop.k));

% Sleepers
Model.Mesh.Mg = funAdd1(Model.Mesh.Mg,Model.Mesh.DOF.sleepers,...
    funDiag(Track.Sleeper.Tnum,Track.Sleeper.Prop.m));

% Ballast on approach to sleepers DOF
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.sleepers_app,...
    funDiag(Track.Sleeper.num_app,Track.Ballast.Prop.c));
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.sleepers_app,...
    funDiag(Track.Sleeper.num_app,Track.Ballast.Prop.k));

% Ballast on bridge to sleepers DOF
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.sleepers_onbeam,...
    funDiag(Track.Sleeper.num_onbeam,Track.BallastOnBeam.Prop.c));
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.sleepers_onbeam,...
    funDiag(Track.Sleeper.num_onbeam,Track.BallastOnBeam.Prop.k));

% Ballast after bridge to sleepers DOF
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.sleepers_aft,...
    funDiag(Track.Sleeper.num_aft,Track.Ballast.Prop.c));
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.sleepers_aft,...
    funDiag(Track.Sleeper.num_aft,Track.Ballast.Prop.k));

% Ballast on approach to Ballast DOF
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.ballast_app,...
    funDiag(Track.Sleeper.num_app,Track.Ballast.Prop.c));
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.ballast_app,...
    funDiag(Track.Sleeper.num_app,Track.Ballast.Prop.k));

% Ballast on bridge to Bridge DOF
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.beam_vert_under_sleeper,...
    funDiag(Track.Sleeper.num_onbeam,Track.BallastOnBeam.Prop.c));
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.beam_vert_under_sleeper,...
    funDiag(Track.Sleeper.num_onbeam,Track.BallastOnBeam.Prop.k));

% Ballast after bridge to Ballast DOF
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.ballast_aft,...
    funDiag(Track.Sleeper.num_aft,Track.Ballast.Prop.c));
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.ballast_aft,...
    funDiag(Track.Sleeper.num_aft,Track.Ballast.Prop.k));

% Ballast on approach
Model.Mesh.Mg = funAdd1(Model.Mesh.Mg,Model.Mesh.DOF.ballast_app,...
    funDiag(Track.Sleeper.num_app,Track.Ballast.Prop.m));

% Ballast on bridge
% Note: The mass of ballast on bridge is distributed to all the Beam's 
%       vertical DOF, rather than only to the ones under a sleeper.
Model.Mesh.Mg = funAdd1(Model.Mesh.Mg,Model.Mesh.DOF.beam_vert,...
    funDiag(Beam.Mesh.Nodes.Tnum,Track.BallastOnBeam.Prop.m/Beam.Mesh.Ele.num_per_spacing));

% Ballast after approach
Model.Mesh.Mg = funAdd1(Model.Mesh.Mg,Model.Mesh.DOF.ballast_aft,...
    funDiag(Track.Sleeper.num_aft,Track.Ballast.Prop.m));

% Beam
Model.Mesh.Mg = funAdd1(Model.Mesh.Mg,Model.Mesh.DOF.beam,Beam.Mesh.Mg);
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.beam,Beam.Mesh.Cg);
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.beam,Beam.Mesh.Kg);

% Sub-Ballast on approach to Ballast DOF
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.ballast_app,...
    funDiag(Track.Sleeper.num_app,Track.SubBallast.Prop.c));
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.ballast_app,...
    funDiag(Track.Sleeper.num_app,Track.SubBallast.Prop.k));

% Ballast after bridge to Ballast DOF
Model.Mesh.Cg = funAdd1(Model.Mesh.Cg,Model.Mesh.DOF.ballast_aft,...
    funDiag(Track.Sleeper.num_aft,Track.SubBallast.Prop.c));
Model.Mesh.Kg = funAdd1(Model.Mesh.Kg,Model.Mesh.DOF.ballast_aft,...
    funDiag(Track.Sleeper.num_aft,Track.SubBallast.Prop.k));

% ---- Off-Diagonal Elementens ----

% Rail and Sleepers
Model.Mesh.Cg = funAdd2(Model.Mesh.Cg,Model.Mesh.DOF.rail_vert_at_sleepers,...
    Model.Mesh.DOF.sleepers,-funDiag(Track.Sleeper.Tnum,Track.Pad.Prop.c));
Model.Mesh.Kg = funAdd2(Model.Mesh.Kg,Model.Mesh.DOF.rail_vert_at_sleepers,...
    Model.Mesh.DOF.sleepers,-funDiag(Track.Sleeper.Tnum,Track.Pad.Prop.k));

% Sleepers and Ballast on approach
Model.Mesh.Cg = funAdd2(Model.Mesh.Cg,Model.Mesh.DOF.sleepers_app,...
    Model.Mesh.DOF.ballast_app,-funDiag(Track.Sleeper.num_app,Track.Ballast.Prop.c));
Model.Mesh.Kg = funAdd2(Model.Mesh.Kg,Model.Mesh.DOF.sleepers_app,...
    Model.Mesh.DOF.ballast_app,-funDiag(Track.Sleeper.num_app,Track.Ballast.Prop.k));

% Sleepers and Beam
Model.Mesh.Cg = funAdd2(Model.Mesh.Cg,Model.Mesh.DOF.sleepers_onbeam,...
    Model.Mesh.DOF.beam_vert_under_sleeper,-funDiag(Track.Sleeper.num_onbeam,Track.BallastOnBeam.Prop.c));
Model.Mesh.Kg = funAdd2(Model.Mesh.Kg,Model.Mesh.DOF.sleepers_onbeam,...
    Model.Mesh.DOF.beam_vert_under_sleeper,-funDiag(Track.Sleeper.num_onbeam,Track.BallastOnBeam.Prop.k));

% Sleepers and Ballast after bridge
Model.Mesh.Cg = funAdd2(Model.Mesh.Cg,Model.Mesh.DOF.sleepers_aft,...
    Model.Mesh.DOF.ballast_aft,-funDiag(Track.Sleeper.num_aft,Track.Ballast.Prop.c));
Model.Mesh.Kg = funAdd2(Model.Mesh.Kg,Model.Mesh.DOF.sleepers_aft,...
    Model.Mesh.DOF.ballast_aft,-funDiag(Track.Sleeper.num_aft,Track.Ballast.Prop.k));

% Sparse matrix output
Model.Mesh.Mg = sparse(Model.Mesh.Mg);
Model.Mesh.Cg = sparse(Model.Mesh.Cg);
Model.Mesh.Kg = sparse(Model.Mesh.Kg);

% Checking symmetry
checksum = sum([max(max(abs(Model.Mesh.Mg-Model.Mesh.Mg'))),...
    max(max(abs(Model.Mesh.Cg-Model.Mesh.Cg'))),...
    max(max(abs(Model.Mesh.Kg-Model.Mesh.Kg')))]);
if checksum > Calc.Cte.tol
    dips('System matrices are not symmetric');
    error('System matrices are not symmetric');
end % if checksum > Calc.Cte.tol

% Auxiliary variables
Model.Mesh.Ele.DOF = Track.Rail.Mesh.Ele.DOF;
Model.Mesh.Ele.a = Track.Rail.Mesh.Ele.a;

% -------------------------- Subfunctions ---------------------------------

function [InM] = funAdd1(InM,ind1,AddM)

    % Subfunction that adds the values of the matrix "AddM" to the 
    % matrix "InM" at rows and columns defined by "ind1"

    InM(ind1,ind1) = InM(ind1,ind1) + AddM;

function [InM] = funAdd2(InM,ind1,ind2,AddM)

    % Subfunction that adds the values of the matrix "AddM" to the 
    % matrix "InM" at rows "ind1" and columns "ind2"
    
    InM(ind1,ind2) = InM(ind1,ind2) + AddM;
    InM(ind2,ind1) = InM(ind2,ind1) + AddM;

function [OutM] = funDiag(size,value)

    % Subfunction that generates a diagonal matrix of dimensions "size"x"size"
    % and with values "value"
    
    OutM = diag(ones(1,size))*value;
    
% ---- End of function ----
