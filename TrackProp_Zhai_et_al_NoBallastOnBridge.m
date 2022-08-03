% Track  Properties

% These vehicle properties are taken from: 
% W.M. Zhai, K.Y. Wang, J.H. Lin, Modelling and experiment of railway 
%   ballast vibrations, Journal of Sound and Vibration, Volume 270, 
%   Issues 4–5, 2004, Pages 673-683, ISSN 0022-460X,
%   https://doi.org/10.1016/S0022-460X(03)00186-X.

% ---- Rail ----
Track.Rail.Prop.E = 2.059e11;       % Young's modulus [N/m^2]
Track.Rail.Prop.I = (3.217e-5)*2;   % Second moment of area [m^4]
Track.Rail.Prop.rho = 60.64*2;      % Mass per unit length [kg/m]
Track.Rail.Damping.per = 0.1;       % Damping [%]

% ---- Pad ----
Track.Pad.Prop.k = 6.5e7;           % Vertical stiffness [N/m]
Track.Pad.Prop.c = 7.5e4;           % Vertical damping [Ns/m]

% ---- Sleeper ----
Track.Sleeper.spacing = 0.6;        % Spacing [m]
Track.Sleeper.Prop.m = 125.5*2;     % Mass [kg]

% ---- Ballast ----
Track.Ballast.Prop.m = 531.4;       % Mass [kg]
Track.Ballast.Prop.k = 137.75e6;    % Stiffness [N/m]
Track.Ballast.Prop.c = 5.88e4;      % Damping [Ns/m]

% ---- SubBallast ----
Track.SubBallast.Prop.k = 77.5e6;   % Stiffness [N/m]
Track.SubBallast.Prop.c = 3.115e4;  % Damping [Ns/m]

% % ---- Ballast on Bridge ----
% Track.BallastOnBeam.Prop.m = Track.Ballast.Prop.m;
% Track.BallastOnBeam.Prop.k = Track.Ballast.Prop.k;
% Track.BallastOnBeam.Prop.c = Track.Ballast.Prop.c;

% ---- Pad under Sleeper on Bridge ----
Track.PadUnderSleeperOnBeam.Prop.k = 120e6;     % Stiffness [N/m]
Track.PadUnderSleeperOnBeam.Prop.c = 60e4;      % Damping [Ns/m]

% ---- End of script ----