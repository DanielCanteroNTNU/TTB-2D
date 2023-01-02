function [Calc] = B19_GenerateProfile(Calc)

% Calculates the irregularity profile for the rail

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
% Calc = Structre with calculation variables. It should include at least:
%   .Profile.Type = Type of Profile 
%       0 = Smooth
%       1 = Generated from custom PSD im [m^3]
% ---- Output ----
% Calc = Additional fields in the structure:
%   ... see script ...
% -------------------------------------------------------------------------

% Profile sampling rate
Calc.Profile.dx = min([min(abs(diff(Calc.Position.x))),Calc.Profile.min_dx]);

Calc.Profile.x = (Calc.Position.x(1):Calc.Profile.dx:Calc.Profile.L);
Calc.Profile.nx = length(Calc.Profile.x);

% ---- Type of Road Profile ----

% -- Smooth --
if Calc.Profile.Type == 0
    
    Calc.Profile.h = zeros(1,Calc.Profile.nx);
    Calc.Profile.text = 'Smooth';
    
% -- From PSD --
elseif Calc.Profile.Type == 1
    
    % [Max,Min] spatial frequency
    min_spaf = 1/Calc.Profile.max_WaveLength;
    max_spaf = 1/Calc.Profile.min_WaveLength;
    
    % Auxiliary variables
    N = 2^nextpow2(Calc.Profile.nx);     % Number of frequencies to calculate

    % PSD X values
    PSD_X = (1/Calc.Profile.dx)*(0:N/2)/N;            % Output
    
    % PSD Y values
    PSD_Y = Calc.Profile.PSD_Y_fun(PSD_X,Calc.Profile.inputs);
    
    % Application of spatial frequency limits
    PSD_Y(PSD_X<min_spaf) = 0;
    PSD_Y(PSD_X>max_spaf) = 0;

    % ---- Generating Output ----
    % Profile elevation
    [Calc.Profile.h] = PSD2profile(PSD_Y,N,Calc.Profile.x);
    % PSD
    Calc.Profile.PSD_X = PSD_X;
    Calc.Profile.PSD_Y = PSD_Y;

end % if Calc.Profile.Type

% ---- Plotting profile ----
if Calc.Plot.Profile_original == 1
    
    % Plotting profile
    figure; 
        plot(Calc.Profile.x,Calc.Profile.h*1000);
        aux1 = get(gcf,'Position'); set(gcf,'Position',[aux1(1:2),500,250]);
        xlim([Calc.Profile.x(1),Calc.Profile.x(end)]);
        xlabel('Distance (m)'); ylabel('Profile Elevation (mm)');
        pause(0.25);

end % if Calc.Plot.Profile_original = 1

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [y] = PSD2profile(PSD_Y,N,X)

    % Auxiliary variables
    dx = X(2) - X(1);
    nx = length(X);
    
    % Two-sided PSD
    PSD2 = PSD_Y/2;
    PSD2([1,end]) = 0;
    PSD2 = [PSD2,fliplr(PSD2(2:end-1))];
    
    % Amplitude
    Amplitude = sqrt(PSD2*N/dx);
    % Random Phase values
    phase = 2*pi*rand(1,N/2-1);
    phase = [0,phase,0,-fliplr(phase)];
    % Fourier transform coefficients
    FFT_y = Amplitude .* exp(1i*phase);
    
    % Inverse Fourier Transform
    iFFT_y = ifft(FFT_y,N);
    
    % Reducing results to desired output
    y = iFFT_y(1:nx);

end % function [y] = PSD2profile(PSD_Y,N,X)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

end % function [Calc] = B19_GenerateProfile(Calc)

% ---- End of function ----
