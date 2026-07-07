function [p_hat, dp_dD] = ISA_model(Down, h0_m)

    % ISA pressure model for a local NED navigation frame.
    %
    % p0 is fixed internally as the ISA sea-level pressure.
    %
    % Inputs:
    %   Down : local NED Down position [m]
    %   h0_m : altitude of the local NED origin above sea level [m]
    %
    % Output:
    %   p_hat : predicted static pressure [Pa]
    %   dp_dD : derivative of pressure with respect to Down position [Pa/m]
    %
    % Local NED altitude relation:
    %
    %   h = h0_m - D
    %
    % ISA pressure model:
    %
    %   p = p0 * (1 - L*h/T0)^alpha
    %
    % Therefore:
    %
    %   dp/dD = p0 * alpha * (1 - L*h/T0)^(alpha - 1) * L/T0
    
    T0 = 288.15;       % sea-level standard temperature [K]
    L  = 0.0065;       % temperature lapse rate [K/m]
    R  = 287.05;       % specific gas constant for air [J/(kg K)]
    g0 = 9.80665;      % gravitational acceleration [m/s^2]
    p0 = 101325.0;     % ISA sea-level pressure [Pa]
    
    alpha = g0 / (R * L);
    
    h = h0_m - Down;
    
    p_hat = p0 * (1.0 - (L * h) / T0)^alpha;
    
    dp_dD = p0 * alpha * (1.0 - (L * h) / T0)^(alpha - 1.0) * (L / T0);

end