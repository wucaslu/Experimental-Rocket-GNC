%% General
g0 = 9.80665;   % gravity [m/s^2]
Sensor.Ts = 1/50; % 50 Hz

%% Gyroscope
Sensor.Gyro.Ts = Sensor.Ts;

% Stochastics noise
Sensor.Gyro.Noise.Brown.Mean = zeros(3,1);
Sensor.Gyro.Noise.Brown.Variance = [1;1;1]*4e-6;

Sensor.Gyro.Noise.White.Mean = zeros(3,1);
Sensor.Gyro.Noise.White.PSD = deg2rad(0.005);    % [rad/s/sqrt(Hz)]
Sensor.Gyro.Noise.White.Variance = ...
    (Sensor.Gyro.Noise.White.PSD)^2 * 1/(2*Sensor.Gyro.Ts);

Sensor.Gyro.Noise.Pink.Mean = zeros(3,1);
Sensor.Gyro.Noise.Pink.Variance = zeros(3,1);

% Systematic errors
Sensor.Gyro.Bias = deg2rad(0.5)*(-1 + 2*rand(3,1));  % bias in [-0.5, 0.5] deg/s
Sensor.Gyro.Delay = 0;
Sensor.Gyro.ScaleFactor = 0.1/100;                   % 0.1% scale-factor error
Sensor.Gyro.NLScaleFactor = 0;
Sensor.Gyro.Assymetry = 0;

% Saturation
Sensor.Gyro.UpperLim = deg2rad(2000);    % +/- 2000 deg/s
Sensor.Gyro.LowerLim = -deg2rad(2000);

% Sensitivity / quantization
Sensor.Gyro.ADC_Bits = 16;
Sensor.Gyro.ADC_Vref = 3.3;
Sensor.Gyro.Sensitivity = Sensor.Gyro.ADC_Vref / ...
    (Sensor.Gyro.UpperLim - Sensor.Gyro.LowerLim);     % [V/(rad/s)]
Sensor.Gyro.Quant_V = Sensor.Gyro.ADC_Vref / 2^Sensor.Gyro.ADC_Bits;
Sensor.Gyro.Quant = Sensor.Gyro.Quant_V / Sensor.Gyro.Sensitivity; % [rad/s]

% Non-orthogonality / alignment error
a = deg2rad(rand(3,1)*2);
b = deg2rad(rand(3,1)*360);

Sensor.Gyro.OR = [ ...
    cos(a(1))              sin(a(1))*cos(b(1))  sin(a(1))*sin(b(1)); ...
    sin(a(2))*cos(b(2))    cos(a(2))            sin(a(2))*sin(b(2)); ...
    sin(a(3))*cos(b(3))    sin(a(3))*sin(b(3))  cos(a(3))];

%% Accelerometer
Sensor.Acc.Ts = Sensor.Ts;

% Stochastics noise
Sensor.Acc.Noise.Brown.Mean = zeros(3,1);
Sensor.Acc.Noise.Brown.Variance = [1;1;1]*2e-4;

Sensor.Acc.Noise.White.Mean = zeros(3,1);
Sensor.Acc.Noise.White.PSD = 100e-6*g0;     % [m/s^2/sqrt(Hz)] 100 ug/sqrt(Hz)
Sensor.Acc.Noise.White.Variance = (Sensor.Acc.Noise.White.PSD)^2 * 1/(2*Sensor.Acc.Ts);

Sensor.Acc.Noise.Pink.Mean = zeros(3,1);
Sensor.Acc.Noise.Pink.Variance = zeros(3,1);

% Systematic errors
Sensor.Acc.Bias = 20e-3*g0*(-1 + 2*rand(3,1));   % bias in [-20 mg, 20 mg]
Sensor.Acc.Delay = 0;
Sensor.Acc.ScaleFactor = 0.1/100;                % 0.1% scale-factor error
Sensor.Acc.NLScaleFactor = 0;
Sensor.Acc.Assymetry = 0;

% Saturation
Sensor.Acc.UpperLim = 16*g0;     % +/- 16 g
Sensor.Acc.LowerLim = -16*g0;

% Sensitivity / quantization
Sensor.Acc.ADC_Bits = 16;
Sensor.Acc.ADC_Vref = 3.3;
Sensor.Acc.Sensitivity = Sensor.Acc.ADC_Vref / ...
    (Sensor.Acc.UpperLim - Sensor.Acc.LowerLim);        % [V/(m/s^2)]
Sensor.Acc.Quant_V = Sensor.Acc.ADC_Vref / 2^Sensor.Acc.ADC_Bits;
Sensor.Acc.Quant = Sensor.Acc.Quant_V / Sensor.Acc.Sensitivity; % [m/s^2]

% Non-orthogonality / alignment error
a = deg2rad(rand(3,1)*2);      % max 2 deg misalignment
b = deg2rad(rand(3,1)*360);

Sensor.Acc.OR = [ ...
    cos(a(1))              sin(a(1))*cos(b(1))  sin(a(1))*sin(b(1)); ...
    sin(a(2))*cos(b(2))    cos(a(2))            sin(a(2))*sin(b(2)); ...
    sin(a(3))*cos(b(3))    sin(a(3))*sin(b(3))  cos(a(3))];

%% Magnetometer
Sensor.Mag.Ts = Sensor.Ts;

% Stochastics noise
Sensor.Mag.Noise.Brown.Mean = zeros(3,1);
Sensor.Mag.Noise.Brown.Variance = zeros(3,1);
Sensor.Mag.Noise.White.Mean = zeros(3,1);
Sensor.Mag.Noise.White.Sigma = 0.4e-6*[1;1;1];
Sensor.Mag.Noise.White.Variance = (Sensor.Mag.Noise.White.Sigma).^2;
Sensor.Mag.Noise.Pink.Mean = zeros(3,1);
Sensor.Mag.Noise.Pink.Variance = zeros(3,1);

% Systematic errors
Sensor.Mag.Bias = 100e-9*(-1 + 2*rand(1))*[1;1;1];  % Bias between [-100e-9, 100e-9]
Sensor.Mag.Delay = 0;
Sensor.Mag.ScaleFactor = 0.01/100;          % 0.01% Linearity Error
Sensor.Mag.NLScaleFactor = 0;
Sensor.Mag.Assymetry = 0;

% Saturation
Sensor.Mag.UpperLim = 1000e-6;
Sensor.Mag.LowerLim = -1000e-6;

% Sensitivity
Sensor.Mag.Sensitivity = 1e6;                                   % sensitivity [V/T]
Sensor.Mag.Quant_V = 10e-3;                                         % quantization interval [V]
Sensor.Mag.Quant = Sensor.Mag.Quant_V/Sensor.Mag.Sensitivity;       % quantization interval [T]

% Non-orthogonality % Assymetry error
a = deg2rad(rand([3,1])*2); % orientation of the non othogonal reference system [rad]
b = deg2rad(rand([3,1])*360);                           
Sensor.Mag.OR=[cos(a(1)) sin(a(1))*cos(b(1)) sin(a(1))*sin(b(1));... % rotation and distortion matrix
       sin(a(2))*cos(b(2)) cos(a(2)) sin(a(2))*sin(b(2));...
       sin(a(3))*cos(b(3)) sin(a(3))*sin(b(3)) cos(a(3))];    

%% Barometer
Sensor.Baro.Ts = Sensor.Ts;

% Stochastics noise
Sensor.Baro.Noise.Brown.Mean = 0;
Sensor.Baro.Noise.Brown.Variance = 0;

Sensor.Baro.Noise.White.Mean = 0;
Sensor.Baro.Noise.White.Sigma = 5;       % [Pa/sqrt(Hz)]
Sensor.Baro.Noise.White.Variance = (Sensor.Baro.Noise.White.Sigma)^2;

Sensor.Baro.Noise.Pink.Mean = 0;
Sensor.Baro.Noise.Pink.Variance = 0;

% Systematic errors
Sensor.Baro.Bias = 10*(-1 + 2*rand(1));  % bias in [-10, 10] Pa
Sensor.Baro.Delay = 0;
Sensor.Baro.ScaleFactor = 2.0e-3;       % 2000 ppm
Sensor.Baro.NLScaleFactor = 0;
Sensor.Baro.Assymetry = 0;

% Saturation
Sensor.Baro.UpperLim = 110000;   % [Pa]
Sensor.Baro.LowerLim = 30000;    % [Pa]

% Sensitivity / quantization
Sensor.Baro.ADC_Bits = 16;
Sensor.Baro.ADC_Vref = 3.3;
Sensor.Baro.Sensitivity = Sensor.Baro.ADC_Vref / ...
    (Sensor.Baro.UpperLim - Sensor.Baro.LowerLim);      % [V/Pa]
Sensor.Baro.Quant_V = Sensor.Baro.ADC_Vref / 2^Sensor.Baro.ADC_Bits;
Sensor.Baro.Quant = Sensor.Baro.Quant_V / Sensor.Baro.Sensitivity; % [Pa]