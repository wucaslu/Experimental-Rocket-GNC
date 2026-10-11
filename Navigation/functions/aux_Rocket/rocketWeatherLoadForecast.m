function [Weather,metadata] = rocketWeatherLoadForecast(Rocket,Weather,Forecast)
%ROCKETWEATHERLOADFORECAST Load and cache an Open-Meteo upper-air forecast.
% Inputs: Rocket, numeric vehicle configuration including latitude/longitude
% [rad] and launch geometric MSL altitude [m]; Weather, existing numeric weather
% configuration; Forecast, initialization options from weatherForecastSettings.
% DateTimeUTC is empty for the current UTC hour, ISO UTC text, or a datetime
% with an explicit timezone. CacheMode is auto, cache (offline), or refresh.
% Outputs: Weather, validated numeric profiles for Environment (SI, MSL/NED);
% metadata, source/valid-time/cache information kept outside block parameters.
% Network and file access happen only here before simulation. No persistent
% variables are used, and forecast errors never silently substitute ISA.
% https://open-meteo.com/en/docs/gfs-api
validateForecastOptions(Forecast);
if ~Forecast.Enabled
    metadata=struct('Source','Configured synthetic weather','Enabled',false);
    return
end
requestedTime=forecastTime(Forecast.DateTimeUTC);
requestedTime.Format='yyyy-MM-dd''T''HH:mm:ss''Z''';
request=struct('SchemaVersion',1,'LatitudeDeg',rad2deg(Rocket.Latitude), ...
    'LongitudeDeg',rad2deg(Rocket.Longitude),'ElevationMSL',Rocket.AltitudeMSL, ...
    'Model',char(Forecast.Model));
assert(all(isfinite([request.LatitudeDeg request.LongitudeDeg request.ElevationMSL])) ...
    && abs(request.LatitudeDeg)<=90 && abs(request.LongitudeDeg)<=180, ...
    'RocketWeatherForecast:InvalidSite','Launch coordinates must be valid WGS84 coordinates.');
cacheMode=char(Forecast.CacheMode);
cacheAvailable=false;
cacheAgeHours=NaN;
if ~strcmp(cacheMode,'refresh') && isfile(Forecast.CacheFile)
    cached=load(Forecast.CacheFile,'ForecastCache');
    if isfield(cached,'ForecastCache')
        record=cached.ForecastCache;
        if isstruct(record) && isscalar(record) && ...
                all(isfield(record,{'Request','Response','FetchedUTC'})) && ...
                isequal(record.Request,request) && isa(record.FetchedUTC,'datetime') ...
                && isscalar(record.FetchedUTC) && ~isnat(record.FetchedUTC) ...
                && ~isempty(record.FetchedUTC.TimeZone)
            cacheAgeHours=hours(datetime('now','TimeZone','UTC')-record.FetchedUTC);
            cacheAvailable=cacheCoversTime(record.Response,requestedTime) && ...
                (strcmp(cacheMode,'cache') || ...
                 (cacheAgeHours>=0 && cacheAgeHours<=Forecast.MaxCacheAgeHours));
        end
    end
end
if strcmp(cacheMode,'cache')
    assert(cacheAvailable,'RocketWeatherForecast:UnavailableCache', ...
        'No matching cached forecast covers %s. Refresh the forecast first.',char(requestedTime));
end
if cacheAvailable
    response=record.Response;
    fetchedTime=record.FetchedUTC;
    fromCache=true;
else
    fields={'surface_pressure','temperature_2m','relative_humidity_2m', ...
        'wind_speed_10m','wind_direction_10m'};
    levels=[1000 975 950 925 900 850 800 700 600 500 400 300 250 200 150 100 70 50 30];
    for k=1:numel(levels)
        for stem={'temperature','relative_humidity','wind_speed', ...
                'wind_direction','geopotential_height'}
            fields{end+1}=sprintf('%s_%dhPa',stem{1},levels(k)); %#ok<AGROW>
        end
    end
    firstHour=dateshift(requestedTime,'start','hour');
    lastHour=firstHour+hours(1);
    firstHour.Format='yyyy-MM-dd''T''HH:mm';
    lastHour.Format=firstHour.Format;
    response=webread('https://api.open-meteo.com/v1/forecast', ...
        'latitude',request.LatitudeDeg,'longitude',request.LongitudeDeg, ...
        'elevation',request.ElevationMSL,'models',request.Model, ...
        'hourly',strjoin(fields,','),'temperature_unit','celsius', ...
        'wind_speed_unit','ms','timezone','GMT','timeformat','iso8601', ...
        'start_hour',char(firstHour),'end_hour',char(lastHour), ...
        weboptions('Timeout',Forecast.TimeoutSeconds));
    fetchedTime=datetime('now','TimeZone','UTC');
    fromCache=false;
    cacheAgeHours=0;
end
% Validate the response and physical profile before replacing a saved cache.
[Weather,metadata]=rocketWeatherForecastProfile(response,requestedTime,Rocket,Weather);
metadata.Enabled=true;
metadata.Model=request.Model;
metadata.RequestedLatitudeDeg=request.LatitudeDeg;
metadata.RequestedLongitudeDeg=request.LongitudeDeg;
metadata.FetchedUTC=char(string(fetchedTime,'yyyy-MM-dd''T''HH:mm:ss''Z'''));
metadata.CacheFile=char(Forecast.CacheFile);
metadata.FromCache=fromCache;
metadata.CacheAgeHours=cacheAgeHours;
metadata.CacheMode=cacheMode;
if ~fromCache
    cacheDirectory=fileparts(Forecast.CacheFile);
    if ~isfolder(cacheDirectory), mkdir(cacheDirectory); end
    ForecastCache=struct('Request',request,'Response',response,'FetchedUTC',fetchedTime);
    save(Forecast.CacheFile,'ForecastCache');
end
end

function t=forecastTime(value)
if (ischar(value) || (isstring(value) && isscalar(value))) && strlength(string(value))==0
    t=dateshift(datetime('now','TimeZone','UTC'),'start','hour');
elseif isa(value,'datetime')
    assert(isscalar(value) && ~isnat(value) && ~isempty(value.TimeZone), ...
        'RocketWeatherForecast:InvalidTime','The input datetime must have an explicit timezone.');
    t=value;
    t.TimeZone='UTC';
else
    assert((ischar(value) && isrow(value)) || (isstring(value) && isscalar(value)), ...
        'RocketWeatherForecast:InvalidTime','Use ISO UTC text or a timezone-aware datetime.');
    value=erase(string(value),'Z');
    assert(~isempty(regexp(value,'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}(:\d{2})?$','once')), ...
        'RocketWeatherForecast:InvalidTime','Use YYYY-MM-DDTHH:MM:SSZ in UTC.');
    if strlength(value)==16, format='yyyy-MM-dd''T''HH:mm';
    else, format='yyyy-MM-dd''T''HH:mm:ss'; end
    t=datetime(value,'InputFormat',format,'TimeZone','UTC');
    assert(~isnat(t),'RocketWeatherForecast:InvalidTime','The requested forecast time is invalid.');
end
end

function covered=cacheCoversTime(response,t)
covered=false;
if ~isstruct(response) || ~isscalar(response) || ~isfield(response,'hourly') ...
        || ~isstruct(response.hourly) || ~isfield(response.hourly,'time')
    return
end
times=datetime(string(response.hourly.time),'InputFormat','yyyy-MM-dd''T''HH:mm', ...
    'TimeZone','UTC');
covered=numel(times)>=2 && all(~isnat(times)) && all(diff(times)>seconds(0)) ...
    && t>=times(1) && t<=times(end);
end

function validateForecastOptions(Forecast)
assert(isstruct(Forecast) && isscalar(Forecast) && ...
    all(isfield(Forecast,{'Enabled','DateTimeUTC','Model','CacheMode', ...
    'MaxCacheAgeHours','TimeoutSeconds','CacheFile'})), ...
    'RocketWeatherForecast:InvalidOptions','Forecast settings are incomplete.');
assert((isnumeric(Forecast.Enabled) || islogical(Forecast.Enabled)) && ...
    isscalar(Forecast.Enabled) && any(Forecast.Enabled==[0 1]), ...
    'RocketWeatherForecast:InvalidOptions','Enabled must be 0 or 1.');
assert(any(strcmp(Forecast.CacheMode,{'auto','cache','refresh'})) && ...
    isscalar(string(Forecast.CacheMode)) && isscalar(string(Forecast.Model)) ...
    && ~isempty(regexp(char(Forecast.Model),'^[a-z][a-z0-9_]*$','once')), ...
    'RocketWeatherForecast:InvalidOptions','Use a valid model name and auto, cache or refresh mode.');
assert(isnumeric(Forecast.MaxCacheAgeHours) && isscalar(Forecast.MaxCacheAgeHours) ...
    && isfinite(Forecast.MaxCacheAgeHours) && Forecast.MaxCacheAgeHours>0 ...
    && isnumeric(Forecast.TimeoutSeconds) && isscalar(Forecast.TimeoutSeconds) ...
    && isfinite(Forecast.TimeoutSeconds) && Forecast.TimeoutSeconds>0 ...
    && isscalar(string(Forecast.CacheFile)) && strlength(string(Forecast.CacheFile))>0, ...
    'RocketWeatherForecast:InvalidOptions','Use positive cache age/timeout and a cache file path.');
end
