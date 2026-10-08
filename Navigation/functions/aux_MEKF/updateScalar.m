function [q, v, p, bg, ba, P] = updateScalar( ...
    z, zhat, H, Rvar, q, v, p, bg, ba, P, estimate_bias)

    r = z - zhat;
    
    S = H * P * H' + Rvar;
    K = (P * H') / S;
    % Frozen biases retain uncertainty; use the same gain for state and Joseph.
    if ~estimate_bias
        K(10:15,:) = 0;
    end
    
    dx = K * r;
    
    I15 = eye(15);
    
    P = (I15 - K * H) * P * (I15 - K * H)' + K * Rvar * K';
    P = 0.5 * (P + P');
    
    [q, v, p, bg, ba, P] = injectError(dx, q, v, p, bg, ba, P, estimate_bias);

end
