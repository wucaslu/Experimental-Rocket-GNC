function metadata=refreshRocketWeatherForecast(dateTimeUTC)
%REFRESHROCKETWEATHERFORECAST Download the latest weather or a specified UTC time.
% Input: optional ISO UTC text or a timezone-aware datetime; omitted/empty uses
% the current UTC hour. Output: metadata describing the validated saved forecast.
% Example: refreshRocketWeatherForecast('2026-10-12T15:30:00Z').
% The saved cache is consumed when rocketPlantMain selects forecast weather
% (RocketWeatherSource='forecast'); Calisto otherwise replays its notebook.
if nargin<1, dateTimeUTC=''; end
navigationDirectory=fileparts(mfilename('fullpath'));
addpath(fullfile(navigationDirectory,'functions','aux_Rocket'));
Rocket=struct(); Weather=struct(); WeatherForecast=struct();
run(fullfile(navigationDirectory,'settings','rocketPlantSettings.m'));
run(fullfile(navigationDirectory,'settings','weatherSettings.m'));
run(fullfile(navigationDirectory,'settings','weatherForecastSettings.m'));
WeatherForecast.Enabled=1;
WeatherForecast.CacheMode='refresh';
WeatherForecast.DateTimeUTC=dateTimeUTC;
[~,metadata]=rocketWeatherLoadForecast(Rocket,Weather,WeatherForecast);
disp(metadata);
end
