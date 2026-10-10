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
Airbrake.BoostSpecificForceUp = 30; % observed boost threshold [m/s^2]
Airbrake.CoastSpecificForceUp = -3; % drag-dominated coast threshold [m/s^2]
Airbrake.MaxUpVelocity = 200;    % upward ground velocity [m/s], strict <
Airbrake.StopDownSpeed = 5;      % latch stop at vUp <= -this value [m/s]
Airbrake.ArmHeightAGL = 20;      % reject stationary/pad velocity noise [m]
Airbrake.ArmUpVelocity = 5;      % require observed ascent [m/s]
Airbrake.ApogeeGain = 1/50;      % deployment per metre of predicted excess
Airbrake.CommandStep = 0.1;     % eleven actuator targets: 0:0.1:1
Airbrake.MaxLevelRate = 0.5;     % deployment/retraction level per second
Airbrake.PredictionEvery = 5;    % update forecast every N IMU samples
Airbrake.PredictionStep = 0.1;   % coast integration step [s]
Airbrake.PredictionMaxSteps = 600;
% Reference memory: [level; ascentSeen; boostSeen; coastLatched; stopLatched;
% counter; forecastAGL; desired]. Simulink stores flags (2:5) upstream and
% law/actuator memory ([1 6 7 8]) separately. There are no flight timers.
AirbrakeInitialState = zeros(8,1);
rocketPlantValidateCdLookup(CdLookup);
assert(all(all(diff(CdLookup.Table,1,2)>=-1e-12)), ...
    'Airbrake:NonmonotoneCd','Airbrake Cd must not decrease as level increases.');
