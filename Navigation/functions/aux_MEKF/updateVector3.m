function [q, v, p, bg, ba, P] = updateVector3( ...
    z, zhat, H, Rvar, q, v, p, bg, ba, P, estimate_bias, ...
    estimate_acc_bias, acc_bias_step_limit, acc_bias_nis_limit)

    % Legacy callers use one enable flag for both bias groups.
    if nargin < 12, estimate_acc_bias = estimate_bias; end
    if nargin < 13, acc_bias_step_limit = inf; end
    if nargin < 14, acc_bias_nis_limit = inf; end

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
    
    dx = K * r;
    
    I15 = eye(15);
    
    P = (I15 - K * H) * P * (I15 - K * H)' + K * R * K';
    P = 0.5 * (P + P');
    
    [q, v, p, bg, ba, P] = injectError( ...
        dx, q, v, p, bg, ba, P, estimate_bias, estimate_acc_bias);

end
