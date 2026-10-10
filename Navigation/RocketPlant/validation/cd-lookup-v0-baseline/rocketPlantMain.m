%% Initialize the physics-driven Navigation model without trajectory files.
rocketNavigationDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rocketNavigationDir,'functions'));
addpath(fullfile(rocketNavigationDir,'functions','aux_math'));
addpath(fullfile(rocketNavigationDir,'functions','aux_MEKF'));
addpath(fullfile(rocketNavigationDir,'functions','aux_GNSS'));
addpath(fullfile(rocketNavigationDir,'functions','aux_Rocket'));
addpath(fullfile(rocketNavigationDir,'settings'));
addpath(fullfile(rocketNavigationDir,'settings','environment'));

run(fullfile(rocketNavigationDir,'settings','sensorSettings.m'));
run(fullfile(rocketNavigationDir,'settings','mekfSettings.m'));
run(fullfile(rocketNavigationDir,'settings','rocketPlantSettings.m'));
run(fullfile(rocketNavigationDir,'settings','weatherSettings.m'));
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
P0 = zeros(15,15);
P0(1:3,1:3) = (5*pi/180)^2*eye(3);
P0(4:6,4:6) = 5^2*eye(3);
P0(7:9,7:9) = 5^2*eye(3);
P0(10:12,10:12) = 0.1^2*eye(3);
P0(13:15,13:15) = 3^2*eye(3);
[pressure0,~] = ISA_model(0,LLA0(3));
