%% Explicit weather configuration for the rocket truth plant (SI units).
% Run after rocketPlantSettings. Defaults preserve the existing dry ISA and
% mean wind exactly. Set UseProfile=1 and rebuild after station/profile edits.
% These editable profiles and deterministic gusts are a synthetic scenario,
% not a measured sounding, weather forecast or validated Dryden turbulence.
Weather = struct();
Weather.UseProfile = 0;
Weather.StationAltitudeMSL = Rocket.AltitudeMSL;      % geometric MSL [m]
[~,weatherStationPressure,~,weatherStationTemperature] = ...
    rocketPlantAtmosphere(Weather.StationAltitudeMSL);
Weather.StationTemperatureK = weatherStationTemperature;
Weather.StationPressurePa = weatherStationPressure;
Weather.StationRelativeHumidity = 0;                % fraction [0,1]
Weather.LapseRateOffset = 0;                        % ISA troposphere + [K/m]
Weather.HumidityScaleHeight = 1500;                 % RH decay above site [m]
Weather.ProfileMinAltitudeMSL = 0;                  % geometric MSL [m]
Weather.ProfileMaxAltitudeMSL = 20000;              % geometric MSL [m]
Weather.ProfileStepAltitude = 25;                   % initialization grid [m]

% Wind is air velocity toward North/East/Down, not meteorological FROM angle.
Weather.WindAltitudeAGL = Rocket.WindAltitude(:);   % above launch site [m]
Weather.WindNED = Rocket.WindNED;                   % knot rows [N E D], [m/s]
Weather.GustEnabled = 1;
Weather.GustRmsNED = zeros(3,1);                    % stationary series RMS [m/s]
Weather.GustFrequenciesHz = [0.05; 0.11; 0.23; 0.47];
Weather.GustWeights = [1; 0.7; 0.5; 0.3];           % L2-normalized at evaluation
Weather.GustPhases = [0 0.7 1.4; 1.1 2.0 0.3; ...   % deterministic radians
                     2.3 0.4 1.8; 0.8 2.7 2.1];
Weather.GustStartTime = Rocket.IgnitionTime;         % absolute model time [s]
Weather.GustRampTime = 5;                           % smooth series onset [s]
Weather.DiscreteGustStartTime = Rocket.IgnitionTime+30;
Weather.DiscreteGustDuration = 20;                  % finite full-cosine pulse [s]
Weather.DiscreteGustPeakNED = zeros(3,1);            % signed peak [m/s]
Weather = rocketWeatherBuildProfile(Weather);
clear weatherStationPressure weatherStationTemperature

% Example weather study (rebuild the profile after editing):
% Weather.UseProfile=1; Weather.StationTemperatureK=Weather.StationTemperatureK+8;
% Weather.StationPressurePa=Weather.StationPressurePa-1200;
% Weather.StationRelativeHumidity=0.45; Weather.LapseRateOffset=0.0005;
% Weather.GustRmsNED=[1.2;0.8;0.15];
% Weather.DiscreteGustPeakNED=[4;-2;0.5];
