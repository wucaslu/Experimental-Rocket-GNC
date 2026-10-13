%% Load the editable Cd CSV once during model initialization.
% CSV headers: Cd,velocity,level. Cd is absolute/dimensionless; velocity is
% CM air-relative speed [m/s]; level is 0:0.1:1. Row order does not matter.
% Supply every velocity/level pair exactly once, with at least two speeds.
% Select your measured/CFD file by changing CdLookupFile below.
% The shipped example is illustrative: a coast-like zero-control curve at
% fixed 340 m/s sound speed, with each 0.1 control step adding 0.05 Cd.
% The table supplies absolute Cd when Rocket.UseCdLookup=1; retracted
% airbrakes use its level-zero column. Calisto analysis sets UseCdLookup=0
% and applies the notebook's powered/coast Mach curves directly instead.
% The rocketpy-airbrakes scenario selects deployment_level,mach,cd source
% data: incremental airbrake drag, added to those nominal powered/coast curves.
cdLookupSettingsDirectory = fileparts(mfilename('fullpath'));
CdLookupFile = fullfile(cdLookupSettingsDirectory,'cd_lookup_example.csv');
if ~exist('RocketParameterSet','var'), RocketParameterSet='calisto'; end
if strcmp(RocketParameterSet,'calisto')
    % Nominal Calisto drag mapped from Mach to speed at 340 m/s sound speed.
    % Deployment increments remain illustrative +0.5*level, not measured.
    CdLookupFile = fullfile(cdLookupSettingsDirectory,'rocketpy_calisto','cd_lookup.csv');
end
if strcmp(RocketParameterSet,'calisto') && exist('RocketScenario','var') && ...
        strcmp(RocketScenario,'rocketpy-airbrakes')
    % Versioned RocketPy source: deployment_level,mach,cd. Cd is the
    % incremental airbrake coefficient, on the rocket reference area.
    CdLookupFile=fullfile(cdLookupSettingsDirectory,'rocketpy_calisto','air_brakes_cd.csv');
end
CdLookup = rocketPlantLoadCdLookupCsv(CdLookupFile);
