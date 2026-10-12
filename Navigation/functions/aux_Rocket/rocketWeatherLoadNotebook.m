function [Weather,metadata] = rocketWeatherLoadNotebook(Rocket,Weather,Source)
%ROCKETWEATHERLOADNOTEBOOK Import the executed RocketPy atmosphere snapshot.
% INPUTS
%   Rocket: launch latitude/longitude [rad] and geometric AltitudeMSL [m].
%       These must match the archived notebook site; no relocation is hidden.
%   Weather: numeric weather configuration supplying runtime gust fields.
%   Source: ProfileFile CSV and MetadataFile JSON paths, initialization only.
% OUTPUTS
%   Weather: numeric geometric-MSL tables for P [Pa], T [K], rho [kg/m^3],
%       sound speed [m/s], RH=0 and wind toward N/E/Down [m/s]. Direct rho/a
%       interpolation replays RocketPy's exported dry atmosphere. No gusts.
%   metadata: snapshot valid time, site, source paths and sampling coverage;
%       strings remain outside the Simulink runtime configuration.
% File access occurs only at initialization; no persistent variables are used.
% The 301 exported heights sample the native forecast at about 20.67 m;
% linear interpolation may smooth native GFS profile slope breakpoints.
assert(isstruct(Source) && isscalar(Source) && ...
    all(isfield(Source,{'ProfileFile','MetadataFile'})) && ...
    isfile(Source.ProfileFile) && isfile(Source.MetadataFile), ...
    'RocketWeatherNotebook:MissingSnapshot','Notebook weather CSV/metadata are required.');
metadata=jsondecode(fileread(Source.MetadataFile));
assert(isstruct(metadata) && isscalar(metadata) && ...
    all(isfield(metadata,{'SchemaVersion','LatitudeDeg','LongitudeDeg', ...
    'ElevationM','ValidTimeUTC'})) && metadata.SchemaVersion==1 && ...
    all(isfinite([metadata.LatitudeDeg metadata.LongitudeDeg metadata.ElevationM])), ...
    'RocketWeatherNotebook:InvalidMetadata','Notebook snapshot metadata are incomplete.');
assert(abs(rad2deg(Rocket.Latitude)-metadata.LatitudeDeg)<1e-8 && ...
    abs(rad2deg(Rocket.Longitude)-metadata.LongitudeDeg)<1e-8 && ...
    abs(Rocket.AltitudeMSL-metadata.ElevationM)<1e-6, ...
    'RocketWeatherNotebook:SiteMismatch', ...
    'Notebook weather requires its launch site and elevation; select forecast weather for another site.');
data=readtable(Source.ProfileFile,'VariableNamingRule','preserve');
columns={'altitude_msl_m','pressure_pa','temperature_k','wind_north_mps', ...
    'wind_east_mps','density_kgpm3','sound_speed_mps','gravity_mps2'};
assert(isequal(data.Properties.VariableNames,columns), ...
    'RocketWeatherNotebook:InvalidColumns','Notebook atmosphere CSV columns/units do not match.');
values=table2array(data);
assert(isnumeric(values) && isreal(values) && size(values,1)>=2 && ...
    all(isfinite(values(:))) && all(diff(values(:,1))>0), ...
    'RocketWeatherNotebook:InvalidData','Notebook profile needs finite increasing geometric-MSL heights.');
Weather.UseProfile=1;
Weather.UseTabulatedThermodynamics=1;
Weather.ProfileAltitudeMSL=values(:,1);
Weather.ProfilePressurePa=values(:,2);
Weather.ProfileTemperatureK=values(:,3);
Weather.ProfileRelativeHumidity=zeros(size(values,1),1);
Weather.ProfileDensityKgpm3=values(:,6);
Weather.ProfileSoundSpeedMps=values(:,7);
Weather.ProfileMinAltitudeMSL=values(1,1);
Weather.ProfileMaxAltitudeMSL=values(end,1);
Weather.ProfileStepAltitude=max(diff(values(:,1)));
Weather.StationAltitudeMSL=Rocket.AltitudeMSL;
Weather.StationPressurePa=interp1(values(:,1),values(:,2),Rocket.AltitudeMSL,'linear');
Weather.StationTemperatureK=interp1(values(:,1),values(:,3),Rocket.AltitudeMSL,'linear');
Weather.StationRelativeHumidity=0;
% The exported columns already contain North/East velocity, not FROM angles.
% Negative AGL knots preserve support below the pad for body-surface loads.
Weather.WindAltitudeAGL=values(:,1)-Rocket.AltitudeMSL;
Weather.WindNED=[values(:,4:5),zeros(size(values,1),1)];
Weather.GustEnabled=0;
Weather.GustRmsNED=zeros(3,1);
Weather.DiscreteGustPeakNED=zeros(3,1);
rocketWeatherValidateConfig(Weather,true);
metadata.Source='Executed RocketPy notebook GFS snapshot';
metadata.Enabled=true;
metadata.ProfileFile=char(Source.ProfileFile);
metadata.MetadataFile=char(Source.MetadataFile);
metadata.ProfileBoundsMSL=[values(1,1),values(end,1)];
metadata.ProfileRows=size(values,1);
metadata.AtmosphericInterpolation='linear P/T/rho/sound; RH=0';
metadata.WindInterpolation='linear North/East velocity; Down=0';
metadata.GustsEnabled=false;
metadata.GravityImported=false; % Earth gravity remains the Environment model.
end
