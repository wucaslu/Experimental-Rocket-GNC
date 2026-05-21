function q = deltaQuat(dtheta)

    angle = norm3(dtheta);
    
    if angle < 1.0e-12
    
        q = [0.5 * dtheta(1);
             0.5 * dtheta(2);
             0.5 * dtheta(3);
             1.0];
    
    else
    
        axis = dtheta / angle;
        half_angle = 0.5 * angle;
    
        q = [axis(1) * sin(half_angle);
             axis(2) * sin(half_angle);
             axis(3) * sin(half_angle);
             cos(half_angle)];
    
    end
    
    q = quatNormalize(q);

end