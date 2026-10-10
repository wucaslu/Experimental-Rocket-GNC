%% First-approach airbrake guidance; target is height above launch [m].
Airbrake.TargetApogeeAGL = 3000;
Airbrake.Enabled = 1;
Airbrake.Automatic = 1;          % 0 selects external level/enable requests
Airbrake.MaxUpVelocity = 200;    % upward ground velocity [m/s], strict <
Airbrake.StopDownSpeed = 5;      % latch stop at vUp <= -this value [m/s]
Airbrake.ArmHeightAGL = 20;      % reject stationary/pad velocity noise [m]
Airbrake.ArmUpVelocity = 5;      % require observed ascent [m/s]
Airbrake.ApogeeGain = 1/50;      % deployment per metre of predicted excess
Airbrake.CommandStep = 0.1;     % actuator targets: 0,0.1,...,1 (nearest step)
Airbrake.MaxLevelRate = 0.5;     % deployment/retraction level per second
Airbrake.PredictionEvery = 5;    % update forecast every N IMU/plant samples
Airbrake.PredictionStep = 0.1;   % coast integration step [s]
Airbrake.PredictionMaxSteps = 600;
% Memory: [actualLevel; ascentSeen; stopLatched; counter; forecastAGL; command].
AirbrakeInitialState = zeros(6,1);
assert(isfinite(Airbrake.CommandStep) && Airbrake.CommandStep>0 && ...
    Airbrake.CommandStep<=1 && ...
    abs(1/Airbrake.CommandStep-round(1/Airbrake.CommandStep))<1e-12, ...
    'Airbrake:InvalidCommandStep','Actuator command step must evenly divide [0,1].');
rocketPlantValidateCdLookup(CdLookup);
assert(all(all(diff(CdLookup.Table,1,2)>=-1e-12)), ...
    'Airbrake:NonmonotoneCd','Airbrake Cd must not decrease as level increases.');
