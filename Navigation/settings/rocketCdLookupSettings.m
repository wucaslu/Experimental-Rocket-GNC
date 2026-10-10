%% Load the editable Cd CSV once during model initialization.
% CSV headers: Cd,velocity,level. Cd is absolute/dimensionless; velocity is
% CM air-relative speed [m/s]; level is 0:0.1:1. Row order does not matter.
% Supply every velocity/level pair exactly once, with at least two speeds.
% Select your measured/CFD file by changing CdLookupFile below.
% The shipped example is illustrative: a coast-like zero-control curve at
% fixed 340 m/s sound speed, with each 0.1 control step adding 0.05 Cd.
% The integrated model always uses this table. Flight gates enable guidance;
% fully retracted airbrakes use the nominal level-zero column at every phase.
% The table supplies absolute Cd and does not switch powered/coast curves.
cdLookupSettingsDirectory = fileparts(mfilename('fullpath'));
CdLookupFile = fullfile(cdLookupSettingsDirectory,'cd_lookup_example.csv');
CdLookup = rocketPlantLoadCdLookupCsv(CdLookupFile);
