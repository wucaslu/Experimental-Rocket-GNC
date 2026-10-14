%% Import the selected Cd calibration once during model initialization.
% IREC defaults to its supplied nominal RASAero curve with control disabled.
% Calisto default: ABCanario.csv, headerless [Level, Mach, Cd]. All entries are
% dimensionless; Cd is TOTAL axial drag, including the level-zero baseline.
% Airspeed is converted to Mach with Environment sound speed for the plant,
% or independent onboard ISA sound speed for Control. The complete grid is
% interpolated bilinearly; Mach/control queries are clipped to its bounds.
% Set RocketCdLookupSource='scenario' before initialization to restore the
% previous scenario-specific example calibration. 'canario' selects this
% user-supplied total-Cd calibration without changing rocket geometry.
% The same numeric CdLookup struct supplies the separate onboard and
% physical lookups. Only initialization reads the file; no persistent state.
cdLookupSettingsDirectory = fileparts(mfilename('fullpath'));
if ~exist('RocketCdLookupSource','var')
    if exist('RocketParameterSet','var') && strcmp(RocketParameterSet,'irec')
        RocketCdLookupSource='irec-nominal';
    else
        RocketCdLookupSource='canario';
    end
end
if strcmp(RocketCdLookupSource,'canario')
    CdLookupFile=fullfile(cdLookupSettingsDirectory,'ABCanario.csv');
    CdLookup=rocketPlantLoadCdLookupCsv(CdLookupFile,'level-mach-cd');
    if exist('Rocket','var')
        Rocket.UseCdLookup=1;
        Rocket.CdLookupIsIncremental=0; % Total Cd must not add nominal drag.
    end
elseif strcmp(RocketCdLookupSource,'irec-nominal')
    assert(exist('RocketParameterSet','var')==1 && strcmp(RocketParameterSet,'irec') && ...
        exist('IrecRocketParameters','var')==1, ...
        'RocketPlant:IrecCdSource','Select the IREC parameter set before loading its nominal drag.');
    % No airbrake calibration was supplied for IREC. A level-independent
    % nominal table keeps both existing lookup interfaces valid at zero
    % command. The rocket uses the original powered/coast Mach curves.
    CdLookupFile=IrecSetupInfo.SourceFiles.Drag;
    CdLookup=struct('Mode',2,'MachBreakpoints',IrecRocketParameters.AeroMach, ...
        'ControlBreakpoints',0:0.1:1, ...
        'Table',repmat(IrecRocketParameters.AeroCdCoast,1,11));
    rocketPlantValidateCdLookup(CdLookup);
    if exist('Rocket','var')
        Rocket.UseCdLookup=0;
        Rocket.CdLookupIsIncremental=0;
    end
elseif strcmp(RocketCdLookupSource,'scenario')
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
    if exist('Rocket','var')
        % Keep the drag convention consistent even when reloading only Cd
        % settings after switching sources in the current workspace.
        Rocket.CdLookupIsIncremental=double(isfield(CdLookup,'Mode') && CdLookup.Mode==1);
    end
else
    error('RocketPlant:InvalidCdLookupSource', ...
        'RocketCdLookupSource must be irec-nominal, canario or scenario.');
end
