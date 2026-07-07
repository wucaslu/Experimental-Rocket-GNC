function [q, v, p, bg, ba, P] = updateVector6( ...
    z, zhat, H, R, q, v, p, bg, ba, P, estimate_bias)

    r = z - zhat;
    
    S = H * P * H' + R;
    K = (P * H') / S;
    
    dx = K * r;
    
    I15 = eye(15);
    
    P = (I15 - K * H) * P * (I15 - K * H)' + K * R * K';
    P = 0.5 * (P + P');
    
    [q, v, p, bg, ba, P] = injectError(dx, q, v, p, bg, ba, P, estimate_bias);

end
