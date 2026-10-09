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
% Match the corrected sensor Brown diffusion (no extra Ts attenuation).
MEKF.sigma_bg = 1.0e-3;
MEKF.sigma_ba = 8.2e-4;

MEKF.sigma_acc_tilt = 5.0e-2; % Normalized direction [-].
% Empirical weights validated with the corrected trajectory and two held-out
% GNSS seeds. They improve launch recovery without changing sensor calibration;
% correlated errors and magnetic reference mismatch still limit covariance accuracy.
MEKF.sigma_mag      = 1.5e-2; % Normalized magnetic direction uncertainty [-].
MEKF.sigma_baro_p   = 10.0;   % Pressure noise [Pa], after startup zeroing.
MEKF.sigma_gps_p    = [1; 1; 5]; % Legacy position [m]; active R uses GNSS fix metadata.
MEKF.acc_gate      = 0.25;    % Fractional gravity-norm gate.

MEKF.sigma_zupt_v = 1.0e-2; % Stationary velocity [m/s].
MEKF.sigma_zaru_g = 3.0e-3; % Stationary angular rate [rad/s]; bias held after launch.
MEKF.sigma_acc_b  = 4.0e-1; % Stationary raw accelerometer [m/s^2].
MEKF.sigma_zp_p   = 1.0e-2; % Stationary local position [m].

% Average measured pressure relative to ISA at the known stationary origin;
% hold the offset after launch. This also absorbs departure from ISA weather.
% Disable when the origin/ISA reference is not justified. This does not
% identify barometer gain or drift, or accelerometer/magnetometer calibration.
MEKF.baro_zero_enabled = true;

% Flight accelerometer bias learning uses GPS position only.
% A single stationary pose cannot separate additive bias from gain/alignment
% errors: ba remains an effective offset until an independent IMU calibration.
MEKF.acc_bias_flight_enabled = true;
MEKF.acc_bias_force_max = 2 * 9.80665; % Specific-force norm [m/s^2].
MEKF.acc_bias_nis_max = 16.3; % Three-dimensional GPS position innovation gate.
MEKF.acc_bias_max_rate = 0.10; % Maximum bias correction norm per second [m/s^3].
MEKF.acc_bias_variance_floor = 0.01; % Effective bias uncertainty [(m/s^2)^2].
