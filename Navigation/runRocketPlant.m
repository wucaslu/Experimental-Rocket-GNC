%% Run the physics-driven model and export its trajectory/validation results.
% Edit settings/rocketPlantSettings.m to supply your rocket. The model's
% InitFcn resolves all source paths and initializes explicit state afresh.
rocketNavigationDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rocketNavigationDir,'RocketPlant'));
addpath(fullfile(rocketNavigationDir,'functions','aux_Rocket'));
in = Simulink.SimulationInput('NavigationRocketPlant');
out = sim(in);
rocketResultsDir = fullfile(rocketNavigationDir,'RocketPlant','validation');
rocketSummary = summarizeRocketPlant(out,Rocket,rocketResultsDir);
weatherSummary = struct();
weatherBaselineFile = fullfile(rocketResultsDir,'aerodynamics-v2','rocket-full-simulation.mat');
if isfile(weatherBaselineFile)
    weatherBaseline = load(weatherBaselineFile,'out','Rocket');
    weatherSummary = summarizeRocketWeather(out,Rocket,Weather,weatherBaseline,rocketResultsDir);
end
save(fullfile(rocketResultsDir,'rocket-full-simulation.mat'),'out','Rocket','Weather', ...
    'rocketSummary','weatherSummary');
disp(rocketSummary);
