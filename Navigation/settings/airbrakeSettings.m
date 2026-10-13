%% First-approach airbrake guidance; target is height above launch [m].
Airbrake = struct(); % Rebuild onboard settings; remove obsolete mode fields.
Airbrake.TargetApogeeAGL = 3000;
Airbrake.Enabled = 1;
% Independent onboard configuration, rather than access to truth plant or
% weather parameters. Update these nominal values for the flight vehicle.
Airbrake.Ts = 0.02;              % controller/IMU sample period [s]
Airbrake.LaunchAltitudeMSL = 901; % surveyed launch-site MSL altitude [m]
Airbrake.DryMass = 18;           % nominal post-burn mass [kg]
Airbrake.ReferenceArea = pi*0.075^2; % reference frontal area [m^2]
if ~exist('RocketParameterSet','var'), RocketParameterSet='calisto'; end
if strcmp(RocketParameterSet,'calisto')
    % Independent onboard constants from the same published vehicle example.
    % No runtime plant-state or wind access is introduced.
    Airbrake.DryMass = 16.241;
    Airbrake.ReferenceArea = pi*0.0635^2;
    Airbrake.Enabled = 0; % current Calisto analysis: retracted airbrakes
    Airbrake.LaunchAltitudeMSL = 1471.4659423828125; % notebook surveyed MSL
end
Airbrake.ControlLawMode = 0;     % 0: target predictor; 1: exact RocketPy; 2: constrained example
Airbrake.ControllerPeriod = 0.1; % 10 Hz example command updates [s]
Airbrake.DeploymentAltitudeAGL = 1500;
Airbrake.DeploymentRate = 0.2;   % example continuous-request limit [level/s]
Airbrake.BoostSpecificForceUp = 30; % observed boost threshold [m/s^2]
Airbrake.CoastSpecificForceUp = -3; % drag-dominated coast threshold [m/s^2]
Airbrake.MaxUpVelocity = 200;    % modes 0/2 only: upward speed [m/s], strict <
Airbrake.StopDownSpeed = 5;      % modes 0/2 only: downward stop threshold [m/s]
Airbrake.ArmHeightAGL = 20;      % reject stationary/pad velocity noise [m]
Airbrake.ArmUpVelocity = 5;      % require observed ascent [m/s]
Airbrake.ApogeeGain = 1/50;      % deployment per metre of predicted excess
Airbrake.CommandStep = 0.1;     % modes 0/2 only: eleven actuator targets
Airbrake.MaxLevelRate = 0.5;     % modes 0/2 only: physical actuator slew [1/s]
Airbrake.PredictionEvery = 5;    % update forecast every N IMU samples
Airbrake.PredictionStep = 0.1;   % coast integration step [s]
Airbrake.PredictionMaxSteps = 600;
% Nominal onboard drag supports the apogee predictor with either table type.
% These fixed published curves are configuration, not runtime truth access.
if strcmp(RocketParameterSet,'calisto')
    nominalDrag=readmatrix(fullfile(fileparts(mfilename('fullpath')), ...
        'rocketpy_calisto','powerOffDragCurve.csv'));
    Airbrake.NominalMach=nominalDrag(:,1);
    Airbrake.NominalCdCoast=nominalDrag(:,2);
end
if strcmp(RocketParameterSet,'calisto') && exist('RocketScenario','var') && ...
        strcmp(RocketScenario,'rocketpy-airbrakes')
    Airbrake.Enabled=1;
    Airbrake.ControlLawMode=1;
    Airbrake.LaunchAltitudeMSL=1400;
    Airbrake.MaxLevelRate=0.2;
end
% Reference memory: [level; ascentSeen; boostSeen; coastLatched; stopLatched;
% counter; forecastAGL; quantizedTarget; continuousRequest; previousVUp].
% Simulink stores flags (2:5), guidance (6:10) and root Actuator memory (1)
% in separate Unit Delays. Mode 1 keeps a global 10 Hz sample phase.
% Mode 1 uses continuous commands and applies deployment directly, with
% +/-0.02 changes at 10 Hz above 1500 m AGL. Burnout is inferred from IMU.
% It has no 200 m/s gate, command bins or downward stop. Mode 2 retains the
% previous constrained comparison for reproducibility.
% There are no flight timers.
AirbrakeInitialState = zeros(10,1);
rocketPlantValidateCdLookup(CdLookup);
