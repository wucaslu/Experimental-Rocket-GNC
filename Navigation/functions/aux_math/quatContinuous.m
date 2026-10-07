function q = quatContinuous(q, q_ref)

    q = quatNormalize(q);
    q_ref = quatNormalize(q_ref);

    % Choose the quaternion representation closest to the reference.
    if q' * q_ref < 0.0
        q = -q;
    end

end