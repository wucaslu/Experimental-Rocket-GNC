%% Apply an external absolute Cd command during early coast.
% Root port 1: Cd_command (dimensionless, nonnegative).
% Root port 2: Cd_override_enabled (0/1).
% Commands are sampled at Rocket.Ts and held during every RK4 stage.
rocketNavigationDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rocketNavigationDir,'RocketPlant'));
run(fullfile(rocketNavigationDir,'rocketPlantMain.m'));

% The motor burns from 50 to 53 s. Override body Cd with 0.9 from 53 to 65 s,
% then return to the configured Mach-dependent coast curve.
cdControlTime = [0;53;65;350];
cdCommand = timeseries(0.9*ones(4,1),cdControlTime);
cdEnable = timeseries([0;1;0;0],cdControlTime);
setinterpmethod(cdCommand,'zoh');
setinterpmethod(cdEnable,'zoh');
cdInputs = Simulink.SimulationData.Dataset;
cdInputs = cdInputs.addElement(cdCommand,'Cd_command');
cdInputs = cdInputs.addElement(cdEnable,'Cd_override_enabled');
in = Simulink.SimulationInput('NavigationRocketPlant');
in = in.setExternalInput(cdInputs);
out = sim(in);

cdExampleDirectory = fullfile(rocketNavigationDir,'RocketPlant','validation', ...
    'cd-control-v1','coast-override');
rocketSummary = summarizeRocketPlant(out,Rocket,cdExampleDirectory);
save(fullfile(cdExampleDirectory,'rocket-full-simulation.mat'), ...
    'out','Rocket','Weather','rocketSummary','cdInputs');
disp(rocketSummary);
