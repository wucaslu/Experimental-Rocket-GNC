function qNED = enuToNedQuaternion(qENU)
%ENUTONEDQUATERNION Convert body-to-ENU attitudes to body-to-NED attitudes.
%   Inputs and outputs are N-by-4 scalar-last Hamilton quaternions [x y z w].
%   Body axes and body angular-rate components are preserved.
%
%   The reference conversion follows the trajectory's two transformations:
%     ENU -> NEU: [0 1 0; 1 0 0; 0 0 1]   (swap East and North)
%     NEU -> NED: diag([1 1 -1])           (Up becomes negative Down)
%   Their product is C_NED_ENU = [0 1 0; 1 0 0; 0 0 -1].
%   NEU is left-handed, so the individual steps are reflections. Only their
%   combined proper rotation is represented by a quaternion.

    arguments
        qENU (:,4) double {mustBeReal, mustBeFinite}
    end

    if any(sum(qENU.^2, 2) < 1e-24)
        error('enuToNedQuaternion:ZeroNorm', ...
            'Attitude quaternions must have nonzero norm.');
    end

    % Combined conversion: 180 degrees about the ENU [1; 1; 0] axis.
    qReference = [sqrt(0.5); sqrt(0.5); 0.0; 0.0];
    qNED = zeros(size(qENU));
    for sample = 1:size(qENU, 1)
        % Left multiplication changes the reference frame, not body axes.
        qNED(sample,:) = quatNormalize( ...
            quatMultiply(qReference, qENU(sample,:)'))';
    end
end
