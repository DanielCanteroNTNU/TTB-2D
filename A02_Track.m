% Definition of Track

% Not a function.
% Defines the content of the variable Track.

% *************************************************************************
% *** Script part of TTB-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% -- Mechanical properties --

% File loading path
Track.Load.path = '';

% Loading predefined list of track properties

% Possibility 1: With Ballast on Bridge
run([Track.Load.path,'TrackProp_Zhai_et_al_WithBallastOnBridge']);

% % Possibility 2: No Ballast on Bridge
% run([Track.Load.path,'TrackProp_Zhai_et_al_NoBallastOnBridge']);

% -- End of script --
