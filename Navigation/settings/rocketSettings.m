%% Rocket settings
% Units: meters, kilograms, seconds, radians.

rocket = struct();

% Geometry and mass
rocket.Sref       = NaN;   % reference area [m^2]
rocket.Lref       = NaN;   % reference length, usually diameter or body length [m]
rocket.mDry       = NaN;   % dry mass [kg]
rocket.mProp0     = NaN;   % initial propellant mass [kg]
rocket.burnTime   = NaN;   % motor burn time [s]

% Inertia matrices
rocket.I_dry_B = [ ...
    NaN 0   0;
    0   NaN 0;
    0   0   NaN];          % dry inertia matrix about dry CM [kg m^2]

rocket.I_prop0_B = [ ...
    NaN 0   0;
    0   NaN 0;
    0   0   NaN];          % initial propellant inertia matrix [kg m^2]

% Reference points in body frame
rocket.rCM_dry_B  = [NaN; NaN; NaN];   % dry center of mass [m]
rocket.rCM_prop_B = [NaN; NaN; NaN];   % propellant center of mass [m]
rocket.rCP_B      = [NaN; NaN; NaN];   % center of pressure [m]
rocket.rNozzle_B  = [NaN; NaN; NaN];   % nozzle/thrust application point [m]

% Thrust direction
rocket.thrustDir_B = [1; 0; 0];        % nominal thrust direction in body frame

% Aerodynamic coefficients
rocket.CNa = NaN;   % normal force slope coefficient [1/rad]
rocket.Clp = NaN;   % roll damping coefficient derivative [-]
rocket.Cmq = NaN;   % pitch damping coefficient derivative [-]
rocket.Cnr = NaN;   % yaw damping coefficient derivative [-]