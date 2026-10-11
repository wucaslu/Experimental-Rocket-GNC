function [q, v, p, bg, ba, P, delta_scale] = updateScalar( ...
    z, zhat, H, Rvar, q, v, p, bg, ba, P, estimate_bias, ...
    estimate_scale, scale_step_bounds)
%UPDATESCALAR Fuse a scalar measurement into the MEKF error state.
% Inputs: measured/predicted scalar z/zhat, Jacobian H, noise variance Rvar,
% nominal q/v/p/bg/ba, covariance P and IMU bias-learning flag. Optional
% estimate_scale enables state 16; scale_step_bounds=[lower;upper] bounds its
% correction and must include zero. Defaults preserve the 15-state API.
% Outputs: corrected nominal states/P and additive scale correction (zero
% for a 15-state covariance or when scale learning is disabled).

    if nargin < 12, estimate_scale = false; end
    if nargin < 13, scale_step_bounds = [0;0]; end

    r = z - zhat;
    
    S = H * P * H' + Rvar;
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
    
    P = (I - K * H) * P * (I - K * H)' + K * Rvar * K';
    P = 0.5 * (P + P');
    
    [q, v, p, bg, ba, P] = injectError(dx, q, v, p, bg, ba, P, estimate_bias);

end
