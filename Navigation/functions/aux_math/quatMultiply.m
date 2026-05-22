function q = quatMultiply(q1, q2)

    % Scalar-last quaternion multiplication:
    %
    %   q = q1 ⊗ q2
    
    v1 = q1(1:3);
    w1 = q1(4);
    
    v2 = q2(1:3);
    w2 = q2(4);
    
    v = w1 * v2 + w2 * v1 + cross3(v1, v2);
    w = w1 * w2 - dot3(v1, v2);
    
    q = [v; w];

end