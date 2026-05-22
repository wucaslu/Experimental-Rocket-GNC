function C = quatToDcm(q)

    % Rotation matrix C_nb from body frame to navigation frame.
    
    q = quatNormalize(q);
    
    x = q(1);
    y = q(2);
    z = q(3);
    w = q(4);
    
    C = zeros(3,3);
    
    C(1,1) = 1.0 - 2.0 * (y*y + z*z);
    C(1,2) = 2.0 * (x*y - z*w);
    C(1,3) = 2.0 * (x*z + y*w);
    
    C(2,1) = 2.0 * (x*y + z*w);
    C(2,2) = 1.0 - 2.0 * (x*x + z*z);
    C(2,3) = 2.0 * (y*z - x*w);
    
    C(3,1) = 2.0 * (x*z - y*w);
    C(3,2) = 2.0 * (y*z + x*w);
    C(3,3) = 1.0 - 2.0 * (x*x + y*y);

end