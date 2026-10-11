function [q, v, p, bg, ba, P, delta_scale] = updateVector3( ...
    z, zhat, H, Rvar, q, v, p, bg, ba, P, estimate_bias, ...
    estimate_acc_bias, acc_bias_step_limit, acc_bias_nis_limit, ...
    estimate_scale, scale_step_bounds)
%UPDATEVECTOR3 Fuse a three-component measurement into the MEKF.
% Inputs: measured/predicted vectors z/zhat, Jacobian H, scalar noise variance
% Rvar, nominal q/v/p/bg/ba, covariance P, gyro/accelerometer learning flags
% and optional accelerometer correction/NIS limits. Optional estimate_scale
% enables state 16 with correction bounds [lower;upper] that include zero.
% Outputs: corrected nominal states/P and additive pressure-scale correction;
% the latter is zero for 15-state callers or disabled scale learning.

    % Legacy callers use one enable flag for both bias groups.
    if nargin < 12, estimate_acc_bias = estimate_bias; end
    if nargin < 13, acc_bias_step_limit = inf; end
    if nargin < 14, acc_bias_nis_limit = inf; end
    if nargin < 15, estimate_scale = false; end
    if nargin < 16, scale_step_bounds = [0;0]; end

    R = Rvar * eye(3);
    
    r = z - zhat;
    
    S = H * P * H' + R;
    K = (P * H') / S;
    % Frozen biases retain uncertainty; use the same gain for state and Joseph.
    if ~estimate_bias
        K(10:12,:) = 0;
    end
    % Reject bias learning from an inconsistent innovation, while allowing
    % position to recover after a bad inertial prediction.
    nis = r' * (S \ r);
    estimate_acc_bias = estimate_acc_bias && isfinite(nis) && ...
        nis <= acc_bias_nis_limit;
    if ~estimate_acc_bias
        K(13:15,:) = 0;
    else
        % Joseph and the state update must use the same bounded gain.
        step_norm = norm(K(13:15,:) * r);
        if step_norm > acc_bias_step_limit
            K(13:15,:) = K(13:15,:) * acc_bias_step_limit / step_norm;
        end
    end
    
    K = mekfLimitScaleGain(K, r, estimate_scale, scale_step_bounds);
    dx = K * r;
    delta_scale = 0;
    if size(P,1) > 15, delta_scale = dx(16); end
    
    I = eye(size(P,1));
    
    P = (I - K * H) * P * (I - K * H)' + K * R * K';
    P = 0.5 * (P + P');
    
    [q, v, p, bg, ba, P] = injectError( ...
        dx, q, v, p, bg, ba, P, estimate_bias, estimate_acc_bias);

end
