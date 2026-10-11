function [q, v, p, bg_out, ba_out, P] = injectError( ...
    dx, q, v, p, bg_in, ba_in, P, estimate_bias, estimate_acc_bias)
%INJECTERROR Apply the MEKF attitude and additive navigation corrections.
% Inputs: dx is the error-state correction; q/v/p and bg_in/ba_in are the
% nominal quaternion, NED velocity/position and IMU biases; P is their error
% covariance, optionally including pressure-scale state 16. The enable flags
% select independent gyro/accelerometer bias learning.
% Outputs: corrected nominal states and covariance after the attitude-error
% reset. An optional scale-state correction is applied by the caller.

    if nargin < 9, estimate_acc_bias = estimate_bias; end

    if ~estimate_bias
        dx(10:12) = zeros(3,1);
    end
    if ~estimate_acc_bias
        dx(13:15) = zeros(3,1);
    end
    dtheta = dx(1:3);
    dv     = dx(4:6);
    dp     = dx(7:9);
    dbg    = dx(10:12);
    dba    = dx(13:15);
    
    % Multiplicative attitude correction.
    dq = deltaQuat(dtheta);
    q = quatNormalize(quatMultiply(q, dq));
    
    % Additive corrections.
    v  = v  + dv;
    p  = p  + dp;
    % Gyro and accelerometer bias learning can be enabled independently.
    if estimate_bias
        bg_out = bg_in + dbg;
    else
        bg_out = bg_in;
    end
    if estimate_acc_bias
        ba_out = ba_in + dba;
    else
        ba_out = ba_in;
    end
    
    % Reset transformation after attitude-error injection.
    Greset = eye(size(P,1));
    Greset(1:3,1:3) = eye(3) - 0.5 * skew3(dtheta);
    
    P = Greset * P * Greset';
    P = 0.5 * (P + P');

end
