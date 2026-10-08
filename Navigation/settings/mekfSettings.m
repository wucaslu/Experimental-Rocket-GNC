%% Active MEKF settings
% Loaded by main.m; MEKF is a Parameter of the active MATLAB Function chart.
% These values describe filter uncertainty, not sensor calibration constants.

% Continuous diffusion amplitudes: Qd = G*diag(sigma.^2)*G'*dt.
% Units are state units / sqrt(s), e.g. gyro rad/sqrt(s),
% accelerometer (m/s)/sqrt(s), gyro bias (rad/s)/sqrt(s).
% Sensor one-sided white PSD uses variance = density^2/(2*Ts),
% so its equivalent diffusion amplitude is density/sqrt(2).
MEKF.sigma_g  = 1.0e-3; % Includes margin for uncalibrated gyro model errors.
MEKF.sigma_a  = 1.5e-1; % Includes the low-frequency colored-noise contribution.
MEKF.sigma_bg = 2.0e-4;
MEKF.sigma_ba = 2.0e-4;

MEKF.sigma_acc_tilt = 5.0e-2; % Normalized direction [-].
MEKF.sigma_mag      = 2.0e-2; % Random direction noise plus calibration margin.
MEKF.sigma_baro_p   = 10.0;   % Pressure noise [Pa], after startup zeroing.
MEKF.sigma_gps_p    = [1; 1; 5]; % Position [m]; active GPS is position-only.
MEKF.acc_gate      = 0.25;    % Fractional gravity-norm gate.

MEKF.sigma_zupt_v = 1.0e-2; % Stationary velocity [m/s].
MEKF.sigma_zaru_g = 5.0e-3; % Stationary angular rate [rad/s].
MEKF.sigma_acc_b  = 4.0e-1; % Stationary raw accelerometer [m/s^2].
MEKF.sigma_zp_p   = 1.0e-2; % Stationary local position [m].

% Average measured pressure relative to ISA at the known stationary origin;
% hold the offset after launch. This also absorbs departure from ISA weather.
% Disable when the origin/ISA reference is not justified. This does not
% identify barometer gain or drift, or accelerometer/magnetometer calibration.
MEKF.baro_zero_enabled = true;
