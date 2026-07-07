function [q, v, p, bg_out, ba_out, P] = injectError(dx, q, v, p, bg_in, ba_in, P, estimate_bias)

    if ~estimate_bias
        dx(10:15) = zeros(6,1);
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
    % Only estimate in stationary mode
    if estimate_bias
        bg_out = bg_in + dbg;
        ba_out = ba_in + dba;
    else
        bg_out = bg_in;
        ba_out = ba_in;
    end
    
    % Reset transformation after attitude-error injection.
    Greset = eye(15);
    Greset(1:3,1:3) = eye(3) - 0.5 * skew3(dtheta);
    
    P = Greset * P * Greset';
    P = 0.5 * (P + P');

end