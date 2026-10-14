function LQR = airbrakeLQRSettings()
%AIRBRAKELQRSETTINGS Return independent onboard coast-controller settings.
% LQR = AIRBRAKELQRSETTINGS() supplies the Canario-tuned finite-horizon,
% affine linear-quadratic tracking design. The controller receives its editable
% apogee target separately; the default target is 3000 m ABOVE LAUNCH.
% No workspace variables, model parameters or persistent state are changed.
% ControllerPeriod and MaxDeploymentRate match the present continuous
% RocketPy-example interface: 10 Hz, at most 0.02 deployment per update.
% Running tolerances scale trajectory perturbations in metres, m/s and
% deployment; terminal height is the objective, with free apogee time.
% The 10 m terminal scale retains nominal peak accuracy with less commanded
% travel than the initial 5 m scale in the Canario nonlinear coast tests.
% Running weights and hardware bounds are unchanged. These settings belong
% to the integrated mode-3 LQR in the saved Simulink Control subsystem.
% Validation: RocketPlant/validation/airbrake-lqr-tuning-canario-v1.
% Coupled installation checks: RocketPlant/validation/lqr-model-integration-v1.
LQR.DefaultTargetAGL = 3000;
LQR.ControllerPeriod = 0.1;        % command update interval [s]
LQR.PredictionStep = 0.1;          % midpoint coast integration step [s]
LQR.MaxSteps = 600;               % bounded 60 s prediction at default step
LQR.MaxDeploymentRate = 0.2;       % continuous deployment rate [1/s]
LQR.TerminalHeightTolerance = 10;  % tuned terminal height scale [m], weight 0.01
LQR.RunningHeightTolerance = 200;  % trajectory-error normalization [m]
LQR.RunningVelocityTolerance = 50; % trajectory-error normalization [m/s]
LQR.RunningLevelTolerance = 1;     % trajectory-error normalization [-]
LQR.RunningWeight = 0.02;          % weak transient tracking relative to peak
LQR.RateTolerance = 0.2;           % rate-effort normalization [1/s]
LQR.ReachabilityTolerance = 0.1;   % boundary decision tolerance [m]
% Differentiate the discrete prediction map. The deployment secant spans
% a physical command increment; microscopic differentiation of the
% scattered Shepard table at zero deployment is poorly conditioned.
LQR.StateJacobianStep = [1;0.5;0.02]; % [m; m/s; deployment]
LQR.RateJacobianStep = 0.02;       % input differentiation increment [1/s]
end
