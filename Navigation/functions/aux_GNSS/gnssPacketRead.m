function [lla, info, lastEpoch, skippedEpochs] = gnssPacketRead( ...
    packet, time_s, lastEpoch, cfg)
%GNSSPACKETREAD Poll an atomic GNSS packet at the navigation/IMU rate.
%   packet = [LLA(3); metadata(8)], with metadata(1) a HELD received-packet
%   marker. Output metadata(1) pulses once per newly observed received epoch.
%   Recompute age at the consumer tick, including rate-transfer waiting.
%   Explicit lastEpoch feedback resets to -1. Faster acquisition uses the
%   latest packet; skippedEpochs counts acquisition epochs not observed.

    assert(isequal(size(packet), [11, 1]));
    assert(isfinite(time_s) && time_s >= 0.0);
    lla = packet(1:3);
    info = packet(4:11);
    skippedEpochs = 0.0;
    tolerance = 1.0e-9 * max(1.0, abs(time_s));
    epoch = info(3);
    received = info(1) > 0.5;
    lossNotification = mod(floor(info(8) / 16.0), 2.0) == 1.0;
    validTime = isfinite(epoch) && epoch >= 0.0 && ...
        epoch <= time_s + tolerance;
    newEpoch = validTime && (received || lossNotification) && ...
        epoch > lastEpoch + tolerance;
    if newEpoch
        if lastEpoch >= 0.0
            skippedEpochs = max(0.0, round((epoch-lastEpoch)/cfg.Ts)-1.0);
        else
            skippedEpochs = max(0.0, round(epoch/cfg.Ts));
        end
        lastEpoch = epoch;
    end
    info(1) = double(newEpoch && received);
    if validTime
        info(4) = max(0.0, time_s - epoch);
    else
        info(4) = 0.0;
        info(2) = 0.0;
    end
    if info(4) > cfg.MaxAge + tolerance
        info(2) = 0.0;
        if mod(floor(info(8) / 32.0), 2.0) == 0.0
            info(8) = info(8) + 32.0;
        end
    end
end
