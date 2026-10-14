%% Select the initialization-time weather source; runtime blocks use tables.
% Calisto replays the frozen atmosphere from the executed RocketPy notebook.
% The generic vehicle retains the configurable Open-Meteo/synthetic source.
% Set RocketWeatherSource='forecast' before initialization to select the
% existing forecast loader. Explicit forecast date/cache inputs also select
% it unless RocketWeatherSource is supplied. No network access occurs for
% 'rocketpy-notebook'; the snapshot valid time is 2026-10-13 12:00 UTC.
WeatherSource = struct();
if strcmp(RocketParameterSet,'calisto')
    WeatherSource.Type = 'rocketpy-notebook';
else
    WeatherSource.Type = 'forecast';
end
WeatherSource.ProfileFile = fullfile(fileparts(mfilename('fullpath')), ...
    'rocketpy_calisto','weather_notebook_gfs.csv');
WeatherSource.MetadataFile = fullfile(fileparts(mfilename('fullpath')), ...
    'rocketpy_calisto','weather_notebook_gfs.json');
if strcmp(RocketParameterSet,'irec')
    WeatherSource.Type='rocketpy-airbrakes';
    WeatherSource.ProfileFile=fullfile(fileparts(mfilename('fullpath')), ...
        'rocketpy_calisto','weather_airbrakes_standard.csv');
    WeatherSource.MetadataFile=fullfile(fileparts(mfilename('fullpath')), ...
        'rocketpy_calisto','weather_airbrakes_standard.json');
end
if strcmp(RocketParameterSet,'calisto') && exist('RocketScenario','var') && ...
        strcmp(RocketScenario,'rocketpy-airbrakes')
    WeatherSource.Type='rocketpy-airbrakes';
    WeatherSource.ProfileFile=fullfile(fileparts(mfilename('fullpath')), ...
        'rocketpy_calisto','weather_airbrakes_standard.csv');
    WeatherSource.MetadataFile=fullfile(fileparts(mfilename('fullpath')), ...
        'rocketpy_calisto','weather_airbrakes_standard.json');
end
