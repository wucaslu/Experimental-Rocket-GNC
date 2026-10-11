function [q, v, p, bg, ba, P, delta_scale] = updateVector6( ...
    z, zhat, H, R, q, v, p, bg, ba, P, estimate_bias, ...
    estimate_scale, scale_step_bounds)
%UPDATEVECTOR6 Fuse a six-component measurement into the MEKF.
% Inputs: measured/predicted vectors z/zhat, Jacobian H, covariance R, nominal
% q/v/p/bg/ba, covariance P and IMU bias-learning flag. Optional estimate_scale
% enables state 16 with correction bounds [lower;upper] that include zero.
% Outputs: corrected nominal states/P and additive pressure-scale correction;
% legacy 15-state callers receive zero for the extra output.

    if nargin < 12, estimate_scale = false; end
    if nargin < 13, scale_step_bounds = [0;0]; end

    r = z - zhat;
    
    S = H * P * H' + R;
    K = (P * H') / S;
    % Frozen biases retain uncertainty; use the same gain for state and Joseph.
    if ~estimate_bias
        K(10:15,:) = 0;
    end
    K = mekfLimitScaleGain(K, r, estimate_scale, scale_step_bounds);
    
    dx = K * r;
    delta_scale = 0;
    if size(P,1) > 15, delta_scale = dx(16); end
    
    I = eye(size(P,1));
    
    P = (I - K * H) * P * (I - K * H)' + K * R * K';
    P = 0.5 * (P + P');
    
    [q, v, p, bg, ba, P] = injectError(dx, q, v, p, bg, ba, P, estimate_bias);

end
