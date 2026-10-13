%% Forecast source for the Environment weather profile (initialization only).
% Used when weatherSourceSettings selects 'forecast'; Calisto defaults to
% the frozen executed-notebook profile instead of a new forecast download.
% Open-Meteo serves NOAA GFS temperature, humidity and wind on pressure levels.
% Reference: https://open-meteo.com/en/docs/gfs-api
% Empty DateTimeUTC uses the current UTC hour from the latest available model.
% To select a forecast valid time, use e.g. '2026-10-12T15:30:00Z' or a
% timezone-aware MATLAB datetime. Dates must be inside the provider's forecast.
% This selects the forecast VALID time, rather than a historical model run.
WeatherForecast = struct();
WeatherForecast.Enabled = 1;
WeatherForecast.DateTimeUTC = '';
WeatherForecast.Model = 'gfs_seamless';
% auto: use a matching fresh local forecast, otherwise download;
% cache: use only the saved forecast, ignoring age (repeatable/offline runs);
% refresh: always download a new forecast before simulation initialization.
WeatherForecast.CacheMode = 'auto';
WeatherForecast.MaxCacheAgeHours = 6;
WeatherForecast.TimeoutSeconds = 30;
WeatherForecast.CacheFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    'RocketPlant','weather','open-meteo-forecast.mat');
