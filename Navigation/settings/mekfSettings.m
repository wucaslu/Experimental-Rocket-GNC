%% MEKF

% -------------------------------------------------------------------------
% Process-noise tuning
% -------------------------------------------------------------------------

sigma_g  = 1.0e-3;  % gyro white noise [rad/s/sqrt(Hz)]
sigma_a  = 2.5e-2;  % accelerometer white noise [m/s^2/sqrt(Hz)]
sigma_bg = 1.0e-4;  % gyro bias random walk [rad/s^2/sqrt(Hz)]
sigma_ba = 1.0e-3;  % accelerometer bias random walk [m/s^3/sqrt(Hz)]

% -------------------------------------------------------------------------
% Measurement-noise tuning
% -------------------------------------------------------------------------

sigma_acc_tilt = 5.0e-2;  % normalized accelerometer direction noise [-]
sigma_mag      = 1.0e-2;  % normalized magnetometer direction noise [-]
sigma_baro_p   = 5;     % barometer pressure noise [Pa]
sigma_gps_p    = [1; 1; 5];     % GPS position noise [m]
sigma_gps_v    = [3.0;3.0;10.0];    % GPS velocity noise [m/s]

%% ZUKF

sigma_zupt_v = 1e-6;    % zero-velocity uncertainty [m/s]
sigma_zaru_g = 5e-3;  % zero-angular-rate uncertainty [rad/s]
sigma_acc_b  = 4e-2;    % stationary accelerometer uncertainty [m/s^2]
sigma_zp_p   = 1e-3;    % zero-position uncertainty [m]