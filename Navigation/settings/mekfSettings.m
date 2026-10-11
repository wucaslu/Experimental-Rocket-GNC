%% Active MEKF settings
% Loaded by main.m and rocketPlantMain.m; MEKF is a chart Parameter.
% These values describe filter uncertainty, not sensor calibration constants.

% Continuous diffusion amplitudes: Qd = G*diag(sigma.^2)*G'*dt.
% Units are state units / sqrt(s), e.g. gyro rad/sqrt(s),
% accelerometer (m/s)/sqrt(s), gyro bias (rad/s)/sqrt(s).
% Sensor one-sided white PSD uses variance = density^2/(2*Ts),
% so its equivalent diffusion amplitude is density/sqrt(2).
% Conservative process uncertainty for the uncalibrated gain/alignment and
% colored-noise residuals; these margins are not identified sensor white PSDs.
MEKF.sigma_g  = 3.0e-3;
MEKF.sigma_a  = 3.0e-1;
% Match the corrected sensor Brown diffusion (no extra Ts attenuation).
MEKF.sigma_bg = 1.0e-3;
MEKF.sigma_ba = 8.2e-4;

MEKF.sigma_acc_tilt = 5.0e-2; % Normalized direction [-].
% Validated through landing with position-only GNSS and frozen flight biases:
% nominal 3 km target, plus a 2.9 km target with stronger wind and gusts.
% Evidence: RocketPlant/validation/mekf-tuning-v1. Correlated errors and the
% order-3 magnetic reference still limit statistical covariance accuracy.
MEKF.sigma_mag      = 3.0e-2; % Normalized magnetic direction uncertainty [-].
MEKF.sigma_baro_p   = 10.0;   % Pressure noise [Pa], after startup zeroing.
MEKF.sigma_gps_p    = [1; 1; 5]; % Legacy position [m]; active R uses GNSS fix metadata.
MEKF.acc_gate      = 0.25;    % Fractional gravity-norm gate.

MEKF.sigma_zupt_v = 1.0e-2; % Stationary velocity [m/s].
MEKF.sigma_zaru_g = 3.0e-3; % Stationary angular rate [rad/s]; bias held after launch.
MEKF.sigma_acc_b  = 4.0e-1; % Stationary raw accelerometer [m/s^2].
MEKF.sigma_zp_p   = 1.0e-2; % Stationary local position [m].

% Average measured pressure relative to ISA at the known stationary origin;
% hold the offset after launch. This removes the initial pressure offset;
% altitude-dependent weather/profile differences remain.
% Disable when the origin/ISA reference is not justified. This does not
% identify barometer gain or drift, or accelerometer/magnetometer calibration.
MEKF.baro_zero_enabled = true;

% GNSS-position-aided pressure-profile adaptation (no forecast inputs).
% P(D,s) = Ppad * (P_ISA(h0-D)/P_ISA(h0))^s. The RocketPlant MEKF
% carries s as error state 16 and explicit nominal state in a Unit Delay.
% Ppad is averaged on the stationary pad and held in flight. Its finite
% averaging uncertainty is not an independent fresh noise at every sample.
% The single scale approximates a profile; the model-noise term below allows
% residual structure, but covariance still requires statistical validation.
% Setting this flag false retains the original fixed-ISA correction for tests.
MEKF.baro_scale_enabled = true;
MEKF.baro_scale_initial = 1.0;         % Dimensionless profile exponent.
MEKF.baro_scale_initial_sigma = 0.10;  % Initial scale standard deviation [-].
MEKF.baro_scale_diffusion = 1.0e-3;    % Continuous diffusion [1/sqrt(s)].
MEKF.baro_scale_min = 0.70;
MEKF.baro_scale_max = 1.30;
MEKF.baro_scale_max_rate = 0.05;       % Bound per correction [1/s].
MEKF.baro_scale_min_height = 100.0;    % Minimum |local Down| for learning [m].
MEKF.baro_scale_max_gnss_gap = 0.50;   % Age of accepted GNSS reference [s].
MEKF.baro_scale_max_gnss_sigma = 20.0; % Maximum reported Down sigma [m].
MEKF.baro_scale_min_pad_samples = 100;
MEKF.baro_scale_pressure_model_sigma = 30.0; % Profile residual allowance [Pa].
MEKF.baro_scale_nis_limit = 25.0;      % One-dimensional pressure outlier gate.

% Estimate both bias groups on the stationary pad and freeze them in flight.
% A single stationary pose cannot separate additive bias from gain/alignment
% errors: ba remains an effective offset until an independent IMU calibration.
MEKF.acc_bias_flight_enabled = false;
% Optional position-only flight-learning limits are inactive with this flag.
MEKF.acc_bias_force_max = 2 * 9.80665; % Specific-force norm [m/s^2].
MEKF.acc_bias_nis_max = 16.3; % Three-dimensional GPS position innovation gate.
MEKF.acc_bias_max_rate = 0.10; % Maximum bias correction norm per second [m/s^3].
MEKF.acc_bias_variance_floor = 0.01; % Effective bias uncertainty [(m/s^2)^2].
