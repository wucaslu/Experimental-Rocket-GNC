function K = mekfLimitScaleGain(K, residual, estimate_scale, scale_step_bounds)
%MEKFLIMITSCALEGAIN Freeze or bound the pressure-scale correction gain.
% Inputs: Kalman gain K, measurement residual, scale-learning enable and
% [minimum;maximum] permitted additive scale correction. Bounds include zero.
% Output: K with only row 16 optionally reduced; the caller must use this gain
% for both state injection and the Joseph covariance update. A 15-state gain
% is returned unchanged. This helper has no retained state.

if size(K,1) <= 15
    return;
end
if ~estimate_scale
    K(16,:) = 0;
    return;
end

lower = scale_step_bounds(1);
upper = scale_step_bounds(2);
step = K(16,:) * residual;
% Invalid bounds or corrections cannot authorize scale-state learning.
if isnan(lower) || isnan(upper) || lower > 0 || upper < 0 || ...
        lower > upper || ~isfinite(step)
    K(16,:) = 0;
elseif step < lower
    K(16,:) = K(16,:) * (lower / step);
elseif step > upper
    K(16,:) = K(16,:) * (upper / step);
end
end
