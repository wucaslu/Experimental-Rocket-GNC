%% Initialize the physics-driven Navigation model without trajectory files.
rocketNavigationDir = fileparts(mfilename('fullpath'));
addpath(rocketNavigationDir);
addpath(fullfile(rocketNavigationDir,'functions'));
addpath(fullfile(rocketNavigationDir,'functions','aux_math'));
addpath(fullfile(rocketNavigationDir,'functions','aux_MEKF'));
addpath(fullfile(rocketNavigationDir,'functions','aux_GNSS'));
addpath(fullfile(rocketNavigationDir,'functions','aux_Rocket'));
addpath(fullfile(rocketNavigationDir,'functions','aux_Motor'));
addpath(fullfile(rocketNavigationDir,'settings'));
addpath(fullfile(rocketNavigationDir,'settings','environment'));

run(fullfile(rocketNavigationDir,'settings','sensorSettings.m'));
run(fullfile(rocketNavigationDir,'settings','mekfSettings.m'));
% Default analysis is the supplied IREC/Mandioca vehicle with its RASAero Cd
% and retracted airbrakes. RocketParameterSet='calisto' restores the previous
% published airbrake example and Canario table. Set RocketCdLookupSource='scenario'
% for the previous drag calibration. RocketScenario='notebook' and
% RocketWeatherSource='rocketpy-notebook' restore the GFS comparison case.
if ~exist('RocketParameterSet','var'),RocketParameterSet='irec';end
if ~exist('RocketScenario','var')
    if strcmp(RocketParameterSet,'irec'),RocketScenario='irec-baseline';
    else,RocketScenario='rocketpy-airbrakes';end
end
run(fullfile(rocketNavigationDir,'settings','rocketPlantSettings.m'));
run(fullfile(rocketNavigationDir,'settings','rocketCdLookupSettings.m'));
run(fullfile(rocketNavigationDir,'settings','weatherSettings.m'));
run(fullfile(rocketNavigationDir,'settings','weatherForecastSettings.m'));
run(fullfile(rocketNavigationDir,'settings','weatherSourceSettings.m'));
% Optional explicit inputs supplied before initialization by the forecast
% callback helper. It restores these temporary inputs on completion or error.
if exist('RocketWeatherForecastDateTimeUTC','var')
    WeatherForecast.DateTimeUTC = RocketWeatherForecastDateTimeUTC;
end
if exist('RocketWeatherForecastCacheMode','var')
    WeatherForecast.CacheMode = RocketWeatherForecastCacheMode;
end
if exist('RocketWeatherSource','var')
    WeatherSource.Type=char(RocketWeatherSource);
elseif exist('RocketWeatherForecastDateTimeUTC','var') || ...
        exist('RocketWeatherForecastCacheMode','var')
    WeatherSource.Type='forecast';
end
switch WeatherSource.Type
    case {'rocketpy-notebook','rocketpy-airbrakes'}
        [Weather,WeatherForecastInfo]=rocketWeatherLoadNotebook(Rocket,Weather,WeatherSource);
    case 'forecast'
        [Weather,WeatherForecastInfo]=rocketWeatherLoadForecast(Rocket,Weather,WeatherForecast);
    otherwise
        error('RocketWeather:InvalidSource', ...
            'RocketWeatherSource must be rocketpy-notebook, rocketpy-airbrakes or forecast.');
end
run(fullfile(rocketNavigationDir,'settings','airbrakeSettings.m'));
Environment.IGRF13.order = 3; % embedded Navigation reference; truth remains 13
[Environment.IGRF13.g, Environment.IGRF13.h] = IGRF13;

stationaryTime = Rocket.IgnitionTime;
marginTime = 5;
RocketInitialState = rocketPlantInitialize(Rocket);
LLA0 = [Rocket.Latitude; Rocket.Longitude; Rocket.AltitudeMSL];
q0 = RocketInitialState(7:10); % body -> NED, scalar-last
run(fullfile(rocketNavigationDir,'settings','gnssDatumSettings.m'));
g0 = 9.80665;
p0 = zeros(3,1);
v0 = zeros(3,1);
ba0 = zeros(3,1);
bg0 = zeros(3,1);
P0 = zeros(16,16); % [attitude; velocity; position; gyro bias; accel bias; s]
P0(1:3,1:3) = (5*pi/180)^2*eye(3);
P0(4:6,4:6) = 5^2*eye(3);
P0(7:9,7:9) = 5^2*eye(3);
P0(10:12,10:12) = 0.1^2*eye(3);
P0(13:15,13:15) = 3^2*eye(3);
P0(16,16) = MEKF.baro_scale_initial_sigma^2;
assert(MEKF.baro_scale_min > 0 && ...
    MEKF.baro_scale_initial >= MEKF.baro_scale_min && ...
    MEKF.baro_scale_initial <= MEKF.baro_scale_max, ...
    'Atmospheric scale must start inside its positive bounds.');
[pressure0,~] = ISA_model(0,LLA0(3));
