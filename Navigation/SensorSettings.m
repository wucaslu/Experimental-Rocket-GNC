%% Sensor Parameters
% Deterministic sensor model for experimental rocket simulation / post-flight analysis.
%
% Datasheets used:
%   [1] Bosch Sensortec, "BMI088 Data Sheet", Rev. 1.9, 2024.
%       Used for gyroscope parameters.
%
%   [2] Analog Devices, "ADXL375: 3-Axis, +/-200 g Digital MEMS
%       Accelerometer Data Sheet".
%       Used for high-dynamic accelerometer parameters.
%
%   [3] STMicroelectronics, "LIS3MDL: Digital Output Magnetic Sensor
%       Datasheet", 2023.
%       Used for magnetometer parameters.
%
%   [4] TE Connectivity / Measurement Specialties,
%       "MS5611-01BA03: Barometric Pressure Sensor, with Stainless Steel Cap".
%       Used for barometer parameters.
%
% Notes:
%   - No random parameters are used in this script.
%   - Biases are deterministic worst-case values with fixed signs.
%   - Misalignment matrices are deterministic small-angle approximations.
%   - Brown and pink noise terms are arbitrary when not available from datasheets.
%   - For a higher-fidelity model, replace the deterministic values with
%     calibration and Allan-variance results from real sensor logs.

%% Constants
if ~exist('g0','var')
    g0 = 9.80665;    % [m/s^2] standard gravity
end

gauss_to_T = 1e-4;   % [T/gauss]

%% General
Sensor.Ts = 1/50;    % [s] 50 Hz common sensor sampling frequency
sec_per_hour = 3600;
sec_per_year = 365.25*24*3600;
%% Deterministic error configuration
% Fixed sign pattern deterministic bias assignment.
sgn3 = [1; -1; 0.5];

% Generic small-angle cross-axis pattern.
E_cross = [ ...
     0   1  -1; ...
    -1   0   1; ...
     1  -1   0  ];

%% Gyroscope
% Datasheet used:
%   Bosch Sensortec BMI088 Data Sheet, Rev. 1.9.
%
% Selected configuration:
%   - Full-scale range: +/- 2000 deg/s
%   - Digital resolution: 16-bit
%   - Noise density: 0.014 deg/s/sqrt(Hz)
%   - Zero-rate offset: approximately +/- 1 deg/s
%   - Sensitivity tolerance: approximately +/- 1%
%   - Sensitivity at +/-2000 deg/s: 16.384 LSB/(deg/s)

Sensor.Gyro.Ts = Sensor.Ts;

% Stochastic noise
Sensor.Gyro.Noise.Brown.Mean = zeros(3,1);
Sensor.Gyro.Noise.Brown.Sigma = 1e-3*ones(3,1);  % [rad/s/sqrt(s)]
Sensor.Gyro.Noise.Brown.Variance = (Sensor.Gyro.Noise.Brown.Sigma).^2;

Sensor.Gyro.Noise.White.Mean = zeros(3,1);
Sensor.Gyro.Noise.White.PSD = deg2rad(0.014);    % [rad/s/sqrt(Hz)]
Sensor.Gyro.Noise.White.Variance = (Sensor.Gyro.Noise.White.PSD)^2 * 1/(2*Sensor.Gyro.Ts);

Sensor.Gyro.Noise.Pink.Mean = zeros(3,1);
Sensor.Gyro.Noise.Pink.Sigma = deg2rad(2/3600)*ones(3,1);     % [rad/s], 2 deg/h
Sensor.Gyro.Noise.Pink.Variance = Sensor.Gyro.Noise.Pink.Sigma.^2;

% Systematic errors
Sensor.Gyro.Bias = deg2rad(0.1)*sgn3;    % fixed bias [rad/s]
Sensor.Gyro.Delay = 0;
Sensor.Gyro.ScaleFactor = 1.0/100;       % 1% scale-factor error
Sensor.Gyro.NLScaleFactor = 0.05/100;    % 0.05% FS nonlinearity
Sensor.Gyro.Assymetry = 0;

% Saturation
Sensor.Gyro.UpperLim = deg2rad(2000);    % [rad/s]
Sensor.Gyro.LowerLim = -deg2rad(2000);   % [rad/s]

% Sensitivity / quantization
Sensor.Gyro.ADC_Bits = 16;
Sensor.Gyro.Sensitivity = 16.384/deg2rad(1);      % [LSB/(rad/s)]
Sensor.Gyro.Quant = 1/Sensor.Gyro.Sensitivity;    % [rad/s/LSB]

% Non-orthogonality / alignment error
Sensor.Gyro.MisalignmentAngle = deg2rad(1.0);     % [rad]
eps_g = Sensor.Gyro.MisalignmentAngle;

Sensor.Gyro.OR = eye(3) + eps_g*E_cross;

%% Accelerometer
% Datasheet used:
%   Analog Devices ADXL375 Data Sheet.
%
% Selected configuration:
%   - Full-scale range: +/- 200 g
%   - Digital output: 16-bit two's complement
%   - Sensitivity: approximately 20.5 LSB/g
%   - Noise density: approximately 5 mg/sqrt(Hz)
%   - Zero-g offset: approximately +/- 400 mg
%   - Nonlinearity: approximately +/- 0.25% FS

Sensor.Acc.Ts = Sensor.Ts;

% Stochastic noise
Sensor.Acc.Noise.Brown.Mean = zeros(3,1);
Sensor.Acc.Noise.Brown.Sigma = ...
    (5e-3*g0/sqrt(sec_per_hour))*ones(3,1);        % [m/s^2/sqrt(s)], 5 mg/sqrt(h)
Sensor.Acc.Noise.Brown.Variance = ...
    (Sensor.Acc.Noise.Brown.Sigma).^2;

Sensor.Acc.Noise.White.Mean = zeros(3,1);
Sensor.Acc.Noise.White.PSD = 5e-3*g0;     % [m/s^2/sqrt(Hz)] 5 mg/sqrt(Hz)
Sensor.Acc.Noise.White.Variance = ...
    (Sensor.Acc.Noise.White.PSD)^2 * 1/(2*Sensor.Acc.Ts);

Sensor.Acc.Noise.Pink.Mean = zeros(3,1);
Sensor.Acc.Noise.Pink.Sigma = ...
    10e-3*g0*ones(3,1);   % [m/s^2], 10 mg
Sensor.Acc.Noise.Pink.Variance = ...
    Sensor.Acc.Noise.Pink.Sigma.^2;

% Systematic errors
Sensor.Acc.Bias = 0.001*sgn3;       % fixed bias [m/s^2], +/-400 mg
Sensor.Acc.Delay = 0;
Sensor.Acc.ScaleFactor = 5/49;       % sensitivity spread approximation
Sensor.Acc.NLScaleFactor = 0.25/100; % 0.25% FS nonlinearity
Sensor.Acc.Assymetry = 0;

% Saturation
Sensor.Acc.UpperLim = 200*g0;        % [m/s^2]
Sensor.Acc.LowerLim = -200*g0;       % [m/s^2]

% Sensitivity / quantization
Sensor.Acc.ADC_Bits = 16;
Sensor.Acc.Sensitivity = 20.5/g0;             % [LSB/(m/s^2)]
Sensor.Acc.Quant = 1/Sensor.Acc.Sensitivity;  % [m/s^2/LSB]

% Non-orthogonality / alignment error
Sensor.Acc.MisalignmentAngle = deg2rad(2.5);  % [rad]
eps_a = Sensor.Acc.MisalignmentAngle;

Sensor.Acc.OR = eye(3) + eps_a*E_cross;

%% Magnetometer
% Datasheet used:
%   STMicroelectronics LIS3MDL Data Sheet.
%
% Selected configuration:
%   - Full-scale range: +/- 16 gauss
%   - Digital output: 16-bit
%   - Sensitivity: 1711 LSB/gauss
%   - RMS noise: approximately 3.2 mgauss for X/Y and 4.1 mgauss for Z
%   - Nonlinearity: approximately +/- 0.12% FS

Sensor.Mag.Ts = Sensor.Ts;

% Stochastic noise
Sensor.Mag.Noise.Brown.Mean = zeros(3,1);
Sensor.Mag.Noise.Brown.Sigma = ...
    (50e-9/sqrt(sec_per_hour))*ones(3,1);          % [T/sqrt(s)], 50 nT/sqrt(h)
Sensor.Mag.Noise.Brown.Variance = ...
    (Sensor.Mag.Noise.Brown.Sigma).^2 * Sensor.Mag.Ts;

Sensor.Mag.Noise.White.Mean = zeros(3,1);
Sensor.Mag.Noise.White.Sigma = [3.2; 3.2; 4.1]*1e-3*gauss_to_T;   % [T RMS]
Sensor.Mag.Noise.White.Variance = (Sensor.Mag.Noise.White.Sigma).^2;

Sensor.Mag.Noise.Pink.Mean = zeros(3,1);
Sensor.Mag.Noise.Pink.Sigma = ...
    100e-9*ones(3,1);                             % [T], 100 nT
Sensor.Mag.Noise.Pink.Variance = ...
    Sensor.Mag.Noise.Pink.Sigma.^2;

% Systematic errors
Sensor.Mag.Bias = 0.01*gauss_to_T*sgn3;   % fixed bias [T], +/-0.01 gauss
Sensor.Mag.Delay = 0;
Sensor.Mag.ScaleFactor = 0;
Sensor.Mag.NLScaleFactor = 0.12/100;     % 0.12% FS nonlinearity
Sensor.Mag.Assymetry = 0;

% Saturation
Sensor.Mag.UpperLim = 16*gauss_to_T;     % [T]
Sensor.Mag.LowerLim = -16*gauss_to_T;    % [T]

% Sensitivity / quantization
Sensor.Mag.ADC_Bits = 16;
Sensor.Mag.Sensitivity = 1711/gauss_to_T;     % [LSB/T]
Sensor.Mag.Quant = 1/Sensor.Mag.Sensitivity;  % [T/LSB]

% Non-orthogonality / alignment error
Sensor.Mag.MisalignmentAngle = deg2rad(2.0);  % [rad]
eps_m = Sensor.Mag.MisalignmentAngle;

Sensor.Mag.OR = eye(3) + eps_m*E_cross;

%% Barometer
% Datasheet used:
%   TE Connectivity / Measurement Specialties MS5611-01BA03 Data Sheet.
%
% Selected configuration:
%   - Pressure range: 10 to 1200 mbar
%   - Digital pressure output: 24-bit ADC
%   - Pressure resolution at OSR = 4096: approximately 0.012 mbar
%   - Typical use: altimeters and variometers

Sensor.Baro.Ts = Sensor.Ts;

% Stochastic noise
Sensor.Baro.Noise.Brown.Mean = 0;
Sensor.Baro.Noise.Brown.Sigma = ...
    100/sqrt(sec_per_year);                       % [Pa/sqrt(s)]

Sensor.Baro.Noise.Brown.Variance = ...
    Sensor.Baro.Noise.Brown.Sigma^2 * Sensor.Baro.Ts;

Sensor.Baro.Noise.White.Mean = 0;
Sensor.Baro.Noise.White.Sigma = 1.2;      % [Pa RMS] = 0.012 mbar
Sensor.Baro.Noise.White.Variance = Sensor.Baro.Noise.White.Sigma^2;

Sensor.Baro.Noise.Pink.Mean = 0;
Sensor.Baro.Noise.Pink.Sigma = 5;                 % [Pa]
Sensor.Baro.Noise.Pink.Variance = ...
    Sensor.Baro.Noise.Pink.Sigma^2;

% Systematic errors
% With one-point calibration, a representative fixed bias of +50 Pa is assumed.
% For the opposite deterministic case, change this value to -50.
Sensor.Baro.Bias = 50;          % [Pa]
Sensor.Baro.Delay = 0;
Sensor.Baro.ScaleFactor = 0;
Sensor.Baro.NLScaleFactor = 0;
Sensor.Baro.Assymetry = 0;

% Saturation
Sensor.Baro.UpperLim = 120000;  % [Pa] = 1200 mbar
Sensor.Baro.LowerLim = 1000;    % [Pa] = 10 mbar

% Sensitivity / quantization
Sensor.Baro.ADC_Bits = 24;
Sensor.Baro.Quant = ...
    (Sensor.Baro.UpperLim - Sensor.Baro.LowerLim)/2^Sensor.Baro.ADC_Bits;  % [Pa/LSB]


%% GPS / GNSS
% Datasheet used:
%   u-blox, "MAX-M10S Data Sheet", UBX-20035208, Rev. R08.
%
% Selected configuration:
%   - GNSS receiver: u-blox MAX-M10S
%   - Constellation mode assumed: GPS + Galileo
%   - Navigation update rate: 10 Hz
%   - Position accuracy: 1.5 m CEP
%   - Velocity accuracy: 0.05 m/s
%   - Operational dynamics limit: <= 4 g
%   - Operational altitude limit: 80000 m
%   - Operational velocity limit: 500 m/s
%
% Notes:
%   - GPS/GNSS is modeled as a low-rate absolute position and velocity sensor.
%   - The measurement frame is assumed to be local NED:
%         position = [p_N; p_E; p_D]
%         velocity = [v_N; v_E; v_D]
%   - The vertical position accuracy is not explicitly specified in the
%     datasheet, so it is conservatively assumed as 2 times the horizontal
%     1-sigma value.
%   - No random bias or random alignment term is used.

Sensor.GPS.Ts = 1/10;     % [s] 10 Hz GNSS update rate

%% GPS stochastic noise

% Datasheet position accuracy is given as CEP.
% For an isotropic 2D Gaussian horizontal error:
%   CEP50 = sigma_xy * sqrt(2*log(2))
% therefore:
%   sigma_xy = CEP50 / sqrt(2*log(2))
Sensor.GPS.PositionAccuracyCEP = 1.5;   % [m]

sigma_xy_gps = Sensor.GPS.PositionAccuracyCEP / sqrt(2*log(2));
sigma_z_gps  = 2*sigma_xy_gps;          % conservative vertical assumption


% Brown noise is usually not used for baseline GNSS position/velocity.
Sensor.GPS.Noise.Brown.Position.Sigma = zeros(3,1);
Sensor.GPS.Noise.Brown.Position.Variance = zeros(3,1);

Sensor.GPS.Noise.Brown.Velocity.Sigma = zeros(3,1);
Sensor.GPS.Noise.Brown.Velocity.Variance = zeros(3,1);

Sensor.GPS.Noise.White.Position.Mean = zeros(3,1);
Sensor.GPS.Noise.White.Position.Sigma = [ ...
    sigma_xy_gps; ...
    sigma_xy_gps; ...
    sigma_z_gps  ];                     % [m]

Sensor.GPS.Noise.White.Position.Variance = ...
    Sensor.GPS.Noise.White.Position.Sigma.^2;

% Velocity accuracy from datasheet.
Sensor.GPS.Noise.White.Velocity.Mean = zeros(3,1);
Sensor.GPS.Noise.White.Velocity.Sigma = 0.05*[1;1;1];   % [m/s]
Sensor.GPS.Noise.White.Velocity.Variance = ...
    Sensor.GPS.Noise.White.Velocity.Sigma.^2;

Sensor.GPS.Noise.Pink.Position.Mean = zeros(3,1);
Sensor.GPS.Noise.Pink.Position.Variance = zeros(3,1);

Sensor.GPS.Noise.Pink.Velocity.Mean = zeros(3,1);
Sensor.GPS.Noise.Pink.Velocity.Variance = zeros(3,1);

% Systematic Errors
% No deterministic bias is introduced by default.
% The position accuracy already represents the dominant GNSS error source.
Sensor.GPS.Position.Bias = zeros(3,1);    % [m]
Sensor.GPS.Velocity.Bias = zeros(3,1);    % [m/s]

% Receiver latency depends on configuration and communication interface.
% It is set to zero here and should be handled by timestamping in the
% navigation architecture.
Sensor.GPS.Delay = 0;                     % [s]

% Scale-factor and nonlinearity are not modeled for GNSS position/velocity.
Sensor.GPS.ScaleFactor = 0;
Sensor.GPS.NLScaleFactor = 0;
Sensor.GPS.Assymetry = 0;

% Operational limits
Sensor.GPS.MaxAcceleration = 4*g0;        % [m/s^2]
Sensor.GPS.MaxAltitude = 80000;           % [m]
Sensor.GPS.MaxVelocity = 500;             % [m/s]

% Saturation / validity limits
% These are used as validity gates, not as hard physical sensor saturation.
Sensor.GPS.Position.UpperLim = [ inf;  inf;  inf];
Sensor.GPS.Position.LowerLim = [-inf; -inf; -inf];

Sensor.GPS.Velocity.UpperLim = Sensor.GPS.MaxVelocity*[1;1;1];
Sensor.GPS.Velocity.LowerLim = -Sensor.GPS.MaxVelocity*[1;1;1];

% Quantization
% GNSS quantization is usually much smaller than the positioning error
% when using binary receiver messages. It is neglected in this preliminary
% simulation model.
Sensor.GPS.Position.Quant = zeros(3,1);   % [m]
Sensor.GPS.Velocity.Quant = zeros(3,1);   % [m/s]

% Measurement covariance
% Measurement vector convention:
%   z_GPS = [p_N; p_E; p_D; v_N; v_E; v_D]
Sensor.GPS.R = diag([ ...
    Sensor.GPS.Noise.White.Position.Variance; ...
    Sensor.GPS.Noise.White.Velocity.Variance ...
]);