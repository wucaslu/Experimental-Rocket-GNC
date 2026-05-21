function euler = quatToEuler321(q)

    % Euler angles using 3-2-1 convention.
    % Output:
    %   euler = [roll; pitch; yaw] [rad]
    
    C = quatToDcm(q);
    
    roll  = atan2(C(3,2), C(3,3));
    pitch = -asin(saturate(C(3,1), -1.0, 1.0));
    yaw   = atan2(C(2,1), C(1,1));
    
    euler = [roll; pitch; yaw];

end