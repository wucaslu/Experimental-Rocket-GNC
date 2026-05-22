function [q, v, p, bg, ba, P] = updateVector3( ...
    z, zhat, H, Rvar, q, v, p, bg, ba, P, estimate_bias)

    R = Rvar * eye(3);
    
    r = z - zhat;
    
    S = H * P * H' + R;
    K = (P * H') / S;
    
    dx = K * r;
    
    I15 = eye(15);
    
    P = (I15 - K * H) * P * (I15 - K * H)' + K * R * K';
    P = 0.5 * (P + P');
    
    [q, v, p, bg, ba, P] = injectError(dx, q, v, p, bg, ba, P, estimate_bias);

end