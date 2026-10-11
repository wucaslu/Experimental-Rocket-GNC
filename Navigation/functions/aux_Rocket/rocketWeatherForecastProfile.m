function [Weather,metadata] = rocketWeatherForecastProfile(response,requestedTimeUTC,Rocket,Weather)
%ROCKETWEATHERFORECASTPROFILE Convert an Open-Meteo forecast into weather tables.
% Inputs:
%   response: decoded forecast JSON, with hourly ISO UTC times, hourly_units,
%     surface P/T/RH/10 m wind and the pressure-level variables listed below.
%   requestedTimeUTC: scalar UTC datetime inside the returned forecast range.
%   Rocket: launch latitude/longitude [rad] and surveyed AltitudeMSL [m].
%   Weather: existing flat numeric configuration; its upper profile altitude
%     and gust settings are retained. Fetching data is the caller's job.
% Outputs:
%   Weather: enabled, station-anchored geometric-MSL atmosphere on a 25 m
%     grid, and AGL/NED mean-wind knots. P [Pa], T [K], RH [0,1], wind [m/s].
%   metadata: source, UTC interpolation bracket, source coverage and explicit
%     5 m hydrostatic lower support / 10 m wind-at-ground approximations.
% This initialization-only converter performs no network/workspace access
% and uses no persistent state. Wind is meteorological FROM in the response;
% interpolate its N/E velocity components in time, never direction angles.
% Pressure-level height is geopotential and is converted to geometric MSL.
% Profiles do not extrapolate above the available pressure-level coverage.

levels=[1000 975 950 925 900 850 800 700 600 500 400 300 250 200 150 100 70 50 30].';
radius=6356766;
support=5;
step=25;
assert(isstruct(response) && isscalar(response) && ...
    isstruct(Rocket) && isscalar(Rocket) && ...
    isstruct(Weather) && isscalar(Weather), ...
    'RocketWeatherForecast:InvalidInput','Response, Rocket and Weather must be scalar structures.');
for name={'Latitude','Longitude','AltitudeMSL'}
    field=name{1};
    assert(isfield(Rocket,field) && finiteScalar(Rocket.(field)), ...
        'RocketWeatherForecast:InvalidInput','Rocket.%s must be a finite real scalar.',field);
end
assert(abs(Rocket.Latitude)<=pi/2 && abs(Rocket.Longitude)<=pi, ...
    'RocketWeatherForecast:InvalidInput','Rocket latitude and longitude must be in radians.');
site=Rocket.AltitudeMSL;
assert(isfield(Weather,'ProfileMaxAltitudeMSL') && ...
    finiteScalar(Weather.ProfileMaxAltitudeMSL) && ...
    Weather.ProfileMaxAltitudeMSL>site && Weather.ProfileMaxAltitudeMSL<=30000 && ...
    site-support>=-5000, ...
    'RocketWeatherForecast:InvalidInput','The requested profile range must contain the surveyed site.');
upper=Weather.ProfileMaxAltitudeMSL;
fields=fieldnames(Weather);
for k=1:numel(fields)
    value=Weather.(fields{k});
    assert((isnumeric(value) || islogical(value)) && isreal(value) && ...
        all(isfinite(value(:))), ...
        'RocketWeatherForecast:InvalidInput','Weather.%s must remain numeric; keep metadata separately.',fields{k});
end
assert(isdatetime(requestedTimeUTC) && isscalar(requestedTimeUTC) && ...
    ~isnat(requestedTimeUTC) && strcmp(requestedTimeUTC.TimeZone,'UTC'), ...
    'RocketWeatherForecast:InvalidTime','requestedTimeUTC must be a scalar datetime with TimeZone UTC.');
if isfield(response,'utc_offset_seconds')
    assert(finiteScalar(response.utc_offset_seconds) && response.utc_offset_seconds==0, ...
        'RocketWeatherForecast:InvalidTime','The response must contain UTC hourly timestamps.');
end
requireField(response,'hourly');
requireField(response,'hourly_units');
assert(isstruct(response.hourly) && isscalar(response.hourly) && ...
    isstruct(response.hourly_units) && isscalar(response.hourly_units), ...
    'RocketWeatherForecast:InvalidInput','hourly and hourly_units must be scalar structures.');
hourly=response.hourly;
units=response.hourly_units;
requireField(hourly,'time');
times=parseTimes(hourly.time);
count=numel(times);
assert(requestedTimeUTC>=times(1) && requestedTimeUTC<=times(end), ...
    'RocketWeatherForecast:OutOfRange','Requested UTC time is outside the returned forecast range.');
exact=find(times==requestedTimeUTC,1);
if ~isempty(exact)
    left=exact;
    right=exact;
    fraction=0;
else
    right=find(times>requestedTimeUTC,1);
    left=right-1;
    fraction=seconds(requestedTimeUTC-times(left))/seconds(times(right)-times(left));
end

surfaceNames={'surface_pressure','temperature_2m','relative_humidity_2m', ...
    'wind_speed_10m','wind_direction_10m'};
surfaceUnits={{'hPa'},{'°C','degC'},{'%'},{'m/s'},{'°','deg'}};
surface=zeros(2,5);
for k=1:numel(surfaceNames)
    surface(:,k)=selectedSeries(hourly,units,surfaceNames{k},surfaceUnits{k},count,left,right);
end
assert(all(isfinite(surface(:))) && all(surface(:,1)>=10 & surface(:,1)<=1200) && ...
    all(surface(:,2)>=-93.15 & surface(:,2)<=76.85) && ...
    all(surface(:,3)>=0 & surface(:,3)<=100) && all(surface(:,4)>=0) && ...
    all(surface(:,5)>=0 & surface(:,5)<=360), ...
    'RocketWeatherForecast:InvalidData','Selected surface pressure, temperature, humidity or wind is invalid.');
stationPressure=blend(surface(:,1),fraction)*100;
stationTemperature=blend(surface(:,2),fraction)+273.15;
stationRH=blend(surface(:,3),fraction)/100;
stationWind=blendWind(surface(:,4),surface(:,5),fraction);

height=zeros(numel(levels),1);
temperature=zeros(numel(levels),1);
humidity=zeros(numel(levels),1);
wind=zeros(numel(levels),3);
above=false(numel(levels),1);
for k=1:numel(levels)
    suffix=sprintf('_%dhPa',levels(k));
    H=selectedSeries(hourly,units,['geopotential_height' suffix],{'m'},count,left,right);
    T=selectedSeries(hourly,units,['temperature' suffix],{'°C','degC'},count,left,right);
    RH=selectedSeries(hourly,units,['relative_humidity' suffix],{'%'},count,left,right);
    speed=selectedSeries(hourly,units,['wind_speed' suffix],{'m/s'},count,left,right);
    direction=selectedSeries(hourly,units,['wind_direction' suffix],{'°','deg'},count,left,right);
    assert(all(isfinite(H)) && all(H<radius) && all(H>-radius), ...
        'RocketWeatherForecast:InvalidData','Selected geopotential heights are invalid at %d hPa.',levels(k));
    interpolatedH=blend(H,fraction);
    height(k)=radius*interpolatedH/(radius-interpolatedH);
    above(k)=height(k)>site;
    % Terrain-buried levels are not atmospheric support for this launch site.
    % Their fields/units must exist, but null thermodynamics may be ignored.
    if ~above(k), continue; end
    assert(all(isfinite([T;RH;speed;direction])) && ...
        all(T>=-93.15 & T<=76.85) && all(RH>=0 & RH<=100) && ...
        all(speed>=0) && all(direction>=0 & direction<=360), ...
        'RocketWeatherForecast:InvalidData','Selected above-site weather is invalid at %d hPa.',levels(k));
    temperature(k)=blend(T,fraction)+273.15;
    humidity(k)=blend(RH,fraction)/100;
    wind(k,:)=blendWind(speed,direction,fraction).';
end
assert(any(above),'RocketWeatherForecast:InsufficientCoverage', ...
    'The forecast has no usable pressure level above the launch site.');
[levelAltitude,order]=sort(height(above));
usedLevels=levels(above);
usedLevels=usedLevels(order);
levelTemperature=temperature(above);
levelTemperature=levelTemperature(order);
levelHumidity=humidity(above);
levelHumidity=levelHumidity(order);
levelWind=wind(above,:);
levelWind=levelWind(order,:);
knotAltitude=[site;levelAltitude];
knotPressure=[stationPressure;usedLevels*100];
assert(all(diff(knotAltitude)>0) && all(diff(knotPressure)<0), ...
    'RocketWeatherForecast:InvalidProfile','Pressure must decrease strictly with geometric altitude above the site.');
assert(levelAltitude(end)>=upper,'RocketWeatherForecast:InsufficientCoverage', ...
    'Forecast coverage ends at %.1f m MSL, below the required %.1f m.',levelAltitude(end),upper);

grid=unique([site-support;(site:step:upper).';upper]);
main=grid>=site;
pressure=zeros(size(grid));
temperatureGrid=zeros(size(grid));
humidityGrid=zeros(size(grid));
pressure(main)=exp(interp1(knotAltitude,log(knotPressure),grid(main),'linear'));
temperatureGrid(main)=interp1(knotAltitude,[stationTemperature;levelTemperature],grid(main),'linear');
humidityGrid(main)=interp1(knotAltitude,[stationRH;levelHumidity],grid(main),'linear');
% Only the 5 m lower support is extrapolated. Constant surface T and RH give
% dp/dH=-g0*(p-pOffset)/(Rd*T), including water vapor partial pressure.
vapor=stationRH*611*10^(7.5*(stationTemperature-273.15)/(237.3+stationTemperature-273.15));
assert(vapor<stationPressure,'RocketWeatherForecast:InvalidProfile', ...
    'Surface water vapor partial pressure must be below total pressure.');
offset=vapor*(1-287.05287/461.5);
stationH=radius*site/(radius+site);
lowerH=radius*grid(~main)./(radius+grid(~main));
pressure(~main)=offset+(stationPressure-offset).* ...
    exp(9.80665*(stationH-lowerH)/(287.05287*stationTemperature));
temperatureGrid(~main)=stationTemperature;
humidityGrid(~main)=stationRH;
% Preserve the surface values exactly at the surveyed station anchor.
station=grid==site;
pressure(station)=stationPressure;
temperatureGrid(station)=stationTemperature;
humidityGrid(station)=stationRH;
Weather.UseProfile=1;
Weather.StationAltitudeMSL=site;
Weather.StationPressurePa=stationPressure;
Weather.StationTemperatureK=stationTemperature;
Weather.StationRelativeHumidity=stationRH;
Weather.ProfileMinAltitudeMSL=site-support;
Weather.ProfileStepAltitude=step;
Weather.ProfileAltitudeMSL=grid;
Weather.ProfilePressurePa=pressure;
Weather.ProfileTemperatureK=temperatureGrid;
Weather.ProfileRelativeHumidity=humidityGrid;
Weather.WindAltitudeAGL=[0;levelAltitude-site];
Weather.WindNED=[stationWind.';levelWind];
try
    rocketWeatherValidateConfig(Weather,true);
catch exception
    error('RocketWeatherForecast:InvalidProfile','Converted weather profile is invalid: %s',exception.message);
end
metadata=struct('Source','Open-Meteo','RequestedTimeUTC',timeText(requestedTimeUTC), ...
    'BracketTimeUTC',{{timeText(times(left)),timeText(times(right))}}, ...
    'TimeInterpolationFraction',fraction, ...
    'LatitudeDegrees',Rocket.Latitude*180/pi,'LongitudeDegrees',Rocket.Longitude*180/pi, ...
    'StationAltitudeMSL',site,'PressureLevelsUsedHpa',usedLevels, ...
    'PressureLevelsIgnoredBelowSiteHpa',levels(~above), ...
    'SourceMaxAltitudeMSL',levelAltitude(end),'ProfileBoundsMSL',[site-support upper], ...
    'GeopotentialRadiusM',radius,'ProfileStepAltitudeM',step, ...
    'AtmosphericInterpolation','log pressure; linear temperature and relative humidity', ...
    'WindTimeInterpolation','North/East velocity components', ...
    'LowerSupportExtensionM',support, ...
    'LowerSupportMethod','isothermal constant-RH hydrostatic extension in geopotential height', ...
    'SurfaceWindReferenceHeightM',10,'SurfaceWindUsedAtGround',true);
end

function yes=finiteScalar(value)
yes=isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end

function requireField(value,name)
assert(isfield(value,name),'RocketWeatherForecast:MissingField', ...
    'Forecast response is missing %s.',name);
end

function values=selectedSeries(hourly,units,name,acceptedUnits,count,left,right)
requireField(hourly,name);
requireField(units,name);
unit=units.(name);
assert((ischar(unit) && isrow(unit)) || (isstring(unit) && isscalar(unit)), ...
    'RocketWeatherForecast:InvalidUnits','Units for %s must be text.',name);
assert(any(strcmp(strtrim(char(unit)),acceptedUnits)), ...
    'RocketWeatherForecast:InvalidUnits','Unsupported units for %s: %s.',name,char(unit));
series=hourly.(name);
% jsondecode represents null in a numeric array as NaN. Select only the
% requested hour bracket, so unavailable distant forecast hours do not leak
% into the selected profile or invalidate otherwise usable launch weather.
assert(isnumeric(series) && isreal(series) && isvector(series) && numel(series)==count, ...
    'RocketWeatherForecast:InvalidData','%s must be a numeric vector matched to hourly.time.',name);
values=[series(left);series(right)];
end

function times=parseTimes(raw)
assert((isstring(raw) || iscellstr(raw) || ischar(raw)) && ~isempty(raw), ...
    'RocketWeatherForecast:InvalidTime','hourly.time must contain ISO UTC timestamps.');
if ischar(raw)
    raw=cellstr(raw);
else
    raw=cellstr(string(raw(:)));
end
times=NaT(numel(raw),1,'TimeZone','UTC');
for k=1:numel(raw)
    value=raw{k};
    assert(~isempty(regexp(value,'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}(:\d{2})?Z?$','once')), ...
        'RocketWeatherForecast:InvalidTime','Hourly time must be ISO UTC without a local offset.');
    if value(end)=='Z', value=value(1:end-1); end
    if numel(value)==16
        format='yyyy-MM-dd''T''HH:mm';
    else
        format='yyyy-MM-dd''T''HH:mm:ss';
    end
    try
        times(k)=datetime(value,'InputFormat',format,'TimeZone','UTC');
    catch
        error('RocketWeatherForecast:InvalidTime','Cannot parse hourly UTC timestamp %s.',value);
    end
end
assert(all(~isnat(times)) && all(diff(times)>seconds(0)), ...
    'RocketWeatherForecast:InvalidTime','Hourly UTC times must be valid and strictly increasing.');
end

function value=blend(pair,fraction)
value=(1-fraction)*pair(1)+fraction*pair(2);
end

function vector=blendWind(speed,direction,fraction)
north=-speed.*cosd(direction);
east=-speed.*sind(direction);
vector=[blend(north,fraction);blend(east,fraction);0];
end

function value=timeText(time)
time.Format='yyyy-MM-dd''T''HH:mm:ss''Z''';
value=char(time);
end
