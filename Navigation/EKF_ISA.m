function [h, v, a, P, info] = EKF_ISA(pBaro, dt, reset)
% EKF_ISA Barometer-only EKF using ISA and analytic pressure derivatives.
%
% Inputs:
%   pBaro : absolute static pressure [Pa]. Use NaN for a missing sample.
%   dt    : time since the previous call [s], finite and nonnegative.
%           The first/reset call only initializes; its dt is not propagated.
%   reset : true for ONE call to discard the previous estimate (optional).
%
% Outputs:
%   h     : geometric pressure altitude [m], positive upward.
%   v     : vertical velocity [m/s], positive upward.
%   a     : kinematic vertical acceleration [m/s^2], positive upward.
%   P     : covariance of [h; v; a].
%   info  : diagnostics; pressureRate [Pa/s], pressureAcceleration [Pa/s^2],
%           ISA spatial derivatives, innovation, NIS, and validity flags.
%
% State and process model:
%   x = [h; v; a], h_dot = v, v_dot = a, a_dot = white jerk.
%   pBaro = p_ISA(h) + measurement noise; H = [dp/dh, 0, 0].
%   p_dot  = (dp/dh)*v.
%   p_ddot = (d2p/dh2)*v^2 + (dp/dh)*a.
%
% Only pressure is assimilated. Numerical derivatives of that same pressure
% are NOT independent measurements. Velocity and acceleration come from the
% pressure time history and the process model; no IMU input is required.
%
% The persistent state allows use in a MATLAB Function block. Before the
% first valid pressure, h/v/a/P are NaN and info.initialized is false.
% Initialization uses its pressure sample exactly once, without propagation
% or a second measurement update. Missing/invalid samples then propagate.
%
% The ISA profile uses geopotential altitude internally, with analytic chain
% derivatives for the GEOMETRIC altitude state. Seven layers are supported
% up to 84852 m geopotential (about 86 km geometric); the first layer is
% extrapolated down to -5000 m geometric. Pressure and dp/dh are continuous
% across layer boundaries; d2p/dh2 uses the selected layer's lapse rate.
%
% Fixed sea-level pressure sets the altitude datum. Unknown weather changes
% or barometer bias cannot be independently resolved with this sensor alone.
% For relative height, subtract the starting altitude from the output.
%
% Reference: U.S. Standard Atmosphere, 1976 (lower-atmosphere ISA profile).
% https://ntrs.nasa.gov/citations/19770009539

if nargin < 3
    reset = false;
end
assert(isscalar(pBaro) && isreal(pBaro), 'pBaro must be a real scalar.');
assert(isscalar(dt) && isreal(dt) && isfinite(dt) && dt >= 0, ...
       'dt must be a finite nonnegative scalar.');
assert(isscalar(reset) && (reset == 0 || reset == 1), ...
       'reset must be false or true.');

% ================= CONSTANTS AND COVARIANCE TUNING ======================
% Starting values: replace these with measured sensor/motion statistics.
SIGMA_PRESSURE = 15.0;       % pressure sample standard deviation [Pa]
JERK_PSD       = 100.0;      % continuous white-jerk PSD [m^2/s^5]
SIGMA_V0       = 10.0;       % initial vertical velocity std [m/s]
SIGMA_A0       = 20.0;       % initial vertical acceleration std [m/s^2]
NIS_GATE       = 25.0;       % 5-sigma scalar innovation gate; Inf disables
P_SEA_LEVEL    = 101325.0;   % fixed reference sea-level pressure [Pa]
% Increase JERK_PSD to follow faster acceleration changes, at more noise.
% R is in Pa^2; dt powers in Q below correspond to continuous white jerk,
% not to an independent constant jerk drawn once per sampling interval.
% ======================================================================

R = SIGMA_PRESSURE^2;
atm = isaConstants(P_SEA_LEVEL);
pValid = isfinite(pBaro) && pBaro > 0.0;

persistent xk Pk initialized
if isempty(initialized) || reset
    xk = zeros(3, 1);
    Pk = zeros(3, 3);
    initialized = false;
end

info = struct('initialized', false, 'updated', false, 'rejected', false, ...
    'pressureValid', pValid, 'modelValid', false, ...
    'pressurePredicted', NaN, 'pressureFiltered', NaN, ...
    'pressureRate', NaN, 'pressureAcceleration', NaN, ...
    'dpdh', NaN, 'd2pdh2', NaN, ...
    'innovation', NaN, 'innovationVariance', NaN, 'nis', NaN);

if ~initialized
    if pValid
        [h0, inverseValid] = isaAltitude(pBaro, atm);
        if inverseValid
            [~, ph0, ~, ~] = isaPressure(h0, atm);
            xk = [h0; 0.0; 0.0];
            % First pressure's uncertainty mapped through the ISA inverse.
            % Fixed pressure datum/model errors are not included in P.
            Pk = diag([R / ph0^2, SIGMA_V0^2, SIGMA_A0^2]);
            initialized = true;
            info.updated = true;
        else
            info.rejected = true;  % outside the initialization model range
        end
    end
else
    % Exact transition for constant acceleration between calls.
    F = [1.0, dt, 0.5*dt^2; ...
         0.0, 1.0, dt; ...
         0.0, 0.0, 1.0];

    % Exact discretization of continuous white noise in a_dot.
    Q = JERK_PSD * [dt^5/20.0, dt^4/8.0, dt^3/6.0; ...
                   dt^4/8.0,  dt^3/3.0, dt^2/2.0; ...
                   dt^3/6.0,  dt^2/2.0, dt];

    xMinus = F*xk;
    PMinus = F*Pk*F.' + Q;
    PMinus = 0.5*(PMinus + PMinus.');

    % Commit prediction even when there is no usable measurement.
    xk = xMinus;
    Pk = PMinus;
    [pMinus, phMinus, ~, modelValid] = isaPressure(xMinus(1), atm);
    info.pressurePredicted = pMinus;

    if pValid && modelValid
        H = [phMinus, 0.0, 0.0];
        innovation = pBaro - pMinus;
        S = H*PMinus*H.' + R;     % scalar variance [Pa^2]
        info.innovation = innovation;
        info.innovationVariance = S;

        if isfinite(S) && S > 0.0 && isfinite(innovation)
            info.nis = innovation^2/S;
            if info.nis <= NIS_GATE
                K = (PMinus*H.')/S;
                xPlus = xMinus + K*innovation;
                [~, ~, ~, correctedValid] = isaPressure(xPlus(1), atm);
                if correctedValid && all(isfinite(xPlus))
                    xk = xPlus;
                    % Joseph form preserves covariance PSD numerically.
                    A = eye(3) - K*H;
                    Pk = A*PMinus*A.' + (K*R)*K.';
                    Pk = 0.5*(Pk + Pk.');
                    info.updated = true;
                else
                    info.rejected = true;
                end
            else
                info.rejected = true;
            end
        else
            info.rejected = true;
        end
    elseif pValid
        % Out-of-domain predictions are reported, never silently clamped.
        info.rejected = true;
    end
end

info.initialized = initialized;
if ~initialized
    h = NaN;
    v = NaN;
    a = NaN;
    P = NaN(3, 3);
    return;
end

h = xk(1);
v = xk(2);
a = xk(3);
P = Pk;
[pHat, ph, phh, modelValid] = isaPressure(h, atm);
info.modelValid = modelValid;
info.pressureFiltered = pHat;
info.dpdh = ph;
info.d2pdh2 = phh;
info.pressureRate = ph*v;
info.pressureAcceleration = ph*a + phh*v^2;
end


function atm = isaConstants(p0)
% All physical/model constants remain in this file.
atm.g = 9.80665;                    % reference gravity [m/s^2]
atm.R = 8314.32/28.9644;            % standard dry-air gas constant [J/(kg K)]
atm.re = 6356766.0;                % geopotential reference Earth radius [m]
atm.Hb = [0.0, 11000.0, 20000.0, 32000.0, ...
          47000.0, 51000.0, 71000.0]; % layer bases, geopotential [m]
atm.L = [-0.0065, 0.0, 0.0010, 0.0028, 0.0, -0.0028, -0.0020];
atm.Tb = zeros(1, 7);
atm.pb = zeros(1, 7);
atm.Tb(1) = 288.15;                % reference sea-level temperature [K]
atm.pb(1) = p0;
atm.hMin = -5000.0;                % geometric lower domain bound [m]
Hmax = 84852.0;                    % geopotential upper domain bound [m]
atm.hMax = atm.re*Hmax/(atm.re - Hmax);

% Generate base pressures recursively: no rounded layer-table jumps.
for j = 1:6
    dH = atm.Hb(j+1) - atm.Hb(j);
    L = atm.L(j);
    atm.Tb(j+1) = atm.Tb(j) + L*dH;
    if L == 0.0
        atm.pb(j+1) = atm.pb(j)*exp(-atm.g*dH/(atm.R*atm.Tb(j)));
    else
        atm.pb(j+1) = atm.pb(j)*exp( ...
            -atm.g/(atm.R*L)*log1p(L*dH/atm.Tb(j)));
    end
end
end


function [p, ph, phh, valid] = isaPressure(h, atm)
% Pressure and first/second spatial derivatives w.r.t. GEOMETRIC altitude.
p = NaN;
ph = NaN;
phh = NaN;
valid = isfinite(h) && h >= atm.hMin && h <= atm.hMax;
if ~valid
    return;
end

Hgeo = atm.re*h/(atm.re + h);        % geopotential altitude [m]
layer = 1;
for j = 2:7
    if Hgeo >= atm.Hb(j)
        layer = j;
    end
end

dH = Hgeo - atm.Hb(layer);
L = atm.L(layer);
Tb = atm.Tb(layer);
T = Tb + L*dH;
if L == 0.0
    p = atm.pb(layer)*exp(-atm.g*dH/(atm.R*Tb));
else
    p = atm.pb(layer)*exp(-atm.g/(atm.R*L)*log1p(L*dH/Tb));
end

% Hydrostatic ISA derivatives in geopotential coordinates.
k = atm.g/atm.R;
pH = -k*p/T;
pHH = k*(k + L)*p/T^2;

% Chain rule from geopotential to geometric altitude.
Hh = (atm.re/(atm.re + h))^2;
Hhh = -2.0*atm.re^2/(atm.re + h)^3;
ph = pH*Hh;
phh = pHH*Hh^2 + pH*Hhh;
end


function [h, valid] = isaAltitude(p, atm)
% Analytic inverse of exactly the same ISA pressure model.
h = NaN;
[pMax, ~, ~, ~] = isaPressure(atm.hMin, atm);
[pMin, ~, ~, ~] = isaPressure(atm.hMax, atm);
valid = isfinite(p) && p >= pMin && p <= pMax;
if ~valid
    return;
end

layer = 1;
for j = 2:7
    if p <= atm.pb(j)
        layer = j;
    end
end

L = atm.L(layer);
Tb = atm.Tb(layer);
logRatio = log(p/atm.pb(layer));
if L == 0.0
    Hgeo = atm.Hb(layer) - atm.R*Tb/atm.g*logRatio;
else
    Hgeo = atm.Hb(layer) + Tb/L*expm1(-atm.R*L/atm.g*logRatio);
end
h = atm.re*Hgeo/(atm.re - Hgeo);
% Allow floating-point roundoff only at the inverse's domain endpoints.
h = min(atm.hMax, max(atm.hMin, h));
end
