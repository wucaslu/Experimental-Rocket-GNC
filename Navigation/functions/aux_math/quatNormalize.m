function q = quatNormalize(q)

    n = sqrt(q(1)*q(1) + q(2)*q(2) + q(3)*q(3) + q(4)*q(4));

    if n < 1.0e-12
        q = [0.0; 0.0; 0.0; 1.0];
    else
        q = q / n;
    end

    % % Enforce a consistent quaternion sign.
    % if q(4) < 0.0
    %     q = -q;
    % end

end