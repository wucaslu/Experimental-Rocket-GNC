function [lla, info, state] = gnssPositionStep( ...
    truthNEU6, time_s, referenceLLArad, cfg, state)
%GNSSPOSITIONSTEP Acquire, perturb and deliver a position-only GNSS fix.
%   truthNEU6 = [north; east; absolute_altitude; vNorth; vEast; vUp], SI.
%   lla = [latitude_deg; longitude_deg; absolute_altitude_m]. The altitude
%   datum is inherited from the trajectory; no geoid correction is implied.
%   info = [fresh; valid; acquisitionTime; age; sigmaN; sigmaE; sigmaD;
%           reasonCode]. Sigmas describe aggregate metric NED position noise.
%   reasonCode is a bit mask: 1 invalid truth, 2 startup/specified outage,
%   4 conservative receiver dynamics gate, 8 reacquisition, 16 packet loss,
%   32 stale. A received invalid-fix packet is fresh but invalid. A packet
%   loss notification is neither fresh nor valid, and never changes LLA.
%
%   Correlated residuals aggregate post-solution atmospheric/orbit/receiver
%   effects. Ground multipath and quality/outlier windows are configurable
%   scenarios, not satellite-geometry predictions. Velocity truth is used
%   only to check the receiver envelope; GNSS velocity is never produced.
%   Call gnssPositionInitialize to reset before each simulation.

    assert(isequal(size(truthNEU6), [6, 1]));
    assert(isfinite(time_s) && time_s >= 0.0);
    tolerance = 1.0e-9 * max(1.0, abs(time_s));
    assert(time_s >= state.previousStepTime - tolerance);
    state.previousStepTime = time_s;

    if time_s >= state.nextAcquisitionTime - tolerance
        % Receiver clock Ts/TransportSubsteps hits every acquisition epoch.
        % If a standalone caller skips ticks after a gap,
        % acquire the current truth once; do not invent historical fixes.
        state.nextAcquisitionTime = ...
            (floor((time_s + tolerance) / cfg.Ts) + 1.0) * cfg.Ts;
        [packetLLA, packetInfo, state] = acquirePosition( ...
            truthNEU6, time_s, referenceLLArad, cfg, state);
        assert(state.queueCount < 32.0);
        tail = mod(state.queueHead + state.queueCount - 1.0, 32.0) + 1.0;
        state.queueLLA(:, tail) = packetLLA;
        state.queueInfo(:, tail) = packetInfo;
        state.queueDueTime(tail) = time_s + cfg.Delay;
        state.queueCount = state.queueCount + 1.0;
    end

    fresh = 0.0;
    % Fixed-size loop is compatible with generated MATLAB Function code.
    for packet = 1:32
        if state.queueCount <= 0.0
            break;
        end
        head = state.queueHead;
        if state.queueDueTime(head) > time_s + tolerance
            break;
        end
        arrived = state.queueInfo(:, head);
        if arrived(2) > 0.5
            state.heldLLA = state.queueLLA(:, head);
        end
        state.heldInfo = arrived;
        fresh = max(fresh, arrived(1));
        state.queueDueTime(head) = realmax;
        state.queueHead = mod(head, 32.0) + 1.0;
        state.queueCount = state.queueCount - 1.0;
    end

    lla = state.heldLLA;
    info = state.heldInfo;
    info(1) = fresh;
    info(4) = max(0.0, time_s - info(3));
    if info(4) > cfg.MaxAge + tolerance
        info(2) = 0.0;
        if mod(floor(info(8) / 32.0), 2.0) == 0.0
            info(8) = info(8) + 32.0;
        end
    end
end

function [lla, info, state] = acquirePosition( ...
    truthNEU6, time_s, referenceLLArad, cfg, state)
    white = zeros(3, 1);
    phiSlow = exp(-cfg.Ts / cfg.Correlated.Tau);
    phiMultipath = exp(-cfg.Ts / cfg.Multipath.Tau);
    for axis = 1:3
        [normalWhite, state.rng(axis)] = normalDraw(state.rng(axis));
        [normalSlow, state.rng(axis + 3)] = ...
            normalDraw(state.rng(axis + 3));
        [normalMultipath, state.rng(axis + 6)] = ...
            normalDraw(state.rng(axis + 6));
        white(axis) = cfg.Noise.White.Position.Sigma(axis) * normalWhite;
        state.slowErrorNED(axis) = phiSlow * state.slowErrorNED(axis) + ...
            cfg.Correlated.Sigma(axis) * sqrt(1.0 - phiSlow^2) * normalSlow;
        state.multipathErrorNED(axis) = ...
            phiMultipath * state.multipathErrorNED(axis) + ...
            cfg.Multipath.Sigma(axis) * ...
            sqrt(1.0 - phiMultipath^2) * normalMultipath;
    end
    [packetDraw, state.rng(10)] = uniformDraw(state.rng(10));

    truthValid = all(isfinite(truthNEU6));
    reason = 0.0;
    altitude = referenceLLArad(3);
    acceleration = 0.0;
    speed = 0.0;
    if truthValid
        altitude = truthNEU6(3);
        speed = sqrt(sum(truthNEU6(4:6).^2));
        if state.previousTruthValid
            elapsed = time_s - state.previousAcquisitionTime;
            if elapsed > 0.0
                acceleration = sqrt(sum((truthNEU6(4:6) - ...
                    state.previousVelocityNEU).^2)) / elapsed;
            end
        end
        state.previousVelocityNEU = truthNEU6(4:6);
    else
        reason = reason + 1.0;
    end
    state.previousTruthValid = truthValid;
    state.previousAcquisitionTime = time_s;

    if time_s < cfg.StartupTime
        reason = reason + 2.0;
    else
        for window = 1:size(cfg.OutageWindows, 1)
            if time_s >= cfg.OutageWindows(window, 1) && ...
                    time_s < cfg.OutageWindows(window, 2)
                reason = reason + 2.0;
                break;
            end
        end
    end
    if truthValid && cfg.EnforceDynamicsLimits && ...
            (acceleration > cfg.MaxAcceleration || ...
             altitude > cfg.MaxAltitude || speed > cfg.MaxVelocity)
        reason = reason + 4.0;
    end
    % StartupTime already represents time to first fix; do not add a second
    % reacquisition interval solely because the receiver was starting up.
    startupOnly = reason == 2.0 && time_s < cfg.StartupTime;
    if reason > 0.0 && ~startupOnly
        state.reacquisitionUntil = time_s + cfg.Ts + cfg.ReacquisitionTime;
    elseif reason == 0.0 && time_s < state.reacquisitionUntil - 1.0e-9
        reason = 8.0;
    end

    quality = 1.0;
    for window = 1:size(cfg.QualityWindows, 1)
        if time_s >= cfg.QualityWindows(window, 1) && ...
                time_s < cfg.QualityWindows(window, 2)
            quality = max(quality, cfg.QualityWindows(window, 3));
        end
    end
    taper = exp(-max(0.0, altitude - referenceLLArad(3)) / ...
        cfg.Multipath.HeightScale);
    sigmaNED = quality * sqrt(cfg.Noise.White.Position.Sigma.^2 + ...
        cfg.Correlated.Sigma.^2 + (taper * cfg.Multipath.Sigma).^2);

    lla = state.heldLLA;
    if reason == 0.0
        errorNED = cfg.Position.Bias + quality * (white + ...
            state.slowErrorNED + taper * state.multipathErrorNED);
        for window = 1:size(cfg.OutlierWindows, 1)
            if time_s >= cfg.OutlierWindows(window, 1) && ...
                    time_s < cfg.OutlierWindows(window, 2)
                errorNED = errorNED + cfg.OutlierWindows(window, 3:5)';
            end
        end
        measuredNEU = truthNEU6(1:3) + ...
            [errorNED(1); errorNED(2); -errorNED(3)];
        lla = gnssNeuToLla(measuredNEU, referenceLLArad);
        for axis = 1:3
            quantum = cfg.Position.QuantLLA(axis);
            if quantum > 0.0
                lla(axis) = round(lla(axis) / quantum) * quantum;
            end
        end
    end
    fresh = 1.0;
    valid = double(reason == 0.0);
    if packetDraw < cfg.PacketLossProbability
        reason = reason + 16.0;
        fresh = 0.0;
        valid = 0.0;
    end
    info = [fresh; valid; time_s; 0.0; sigmaNED; reason];
end

function [normal, seed] = normalDraw(seed)
    [uniform1, seed] = uniformDraw(seed);
    [uniform2, seed] = uniformDraw(seed);
    normal = sqrt(-2.0 * log(uniform1)) * cos(2.0 * pi * uniform2);
end

function [uniform, seed] = uniformDraw(seed)
    % Integer arithmetic stays exact within IEEE double's 53-bit mantissa.
    seed = mod(16807.0 * seed, 2147483647.0);
    uniform = seed / 2147483647.0;
end
