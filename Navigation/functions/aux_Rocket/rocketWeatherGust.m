function gustNED = rocketWeatherGust(t,Weather)
%ROCKETWEATHERGUST Deterministic explicit-time synthetic gust velocity [m/s].
% Fourier amplitudes give the configured per-axis infinite-time RMS after
% the smooth onset ramp; a finite flight window need not have that exact RMS.
% The independent full-cosine pulse is zero with zero slope at both endpoints.
% This finite time pulse is an original adaptation of the 1-cosine concept,
% not the distance-based military gust or a stochastic Dryden spectrum.
% Fixed phases preserve repeatability without RNG, persistent data or state.
% https://www.mathworks.com/help/aeroblks/discretewindgustmodel.html
gustNED=zeros(3,1);
if nargin<2 || isempty(Weather) || Weather.GustEnabled==0, return; end
if all(Weather.GustRmsNED==0) && all(Weather.DiscreteGustPeakNED==0), return; end
elapsed=t-Weather.GustStartTime;
if elapsed>0
    ramp=1;
    if elapsed<Weather.GustRampTime
        ramp=0.5*(1-cos(pi*elapsed/Weather.GustRampTime));
    end
    weights=Weather.GustWeights/norm(Weather.GustWeights);
    for k=1:numel(weights)
        for axis=1:3
            gustNED(axis)=gustNED(axis)+ramp*sqrt(2)* ...
                Weather.GustRmsNED(axis)*weights(k)* ...
                sin(2*pi*Weather.GustFrequenciesHz(k)*elapsed+Weather.GustPhases(k,axis));
        end
    end
end
tau=(t-Weather.DiscreteGustStartTime)/Weather.DiscreteGustDuration;
if tau>0 && tau<1
    gustNED=gustNED+0.5*(1-cos(2*pi*tau))*Weather.DiscreteGustPeakNED;
end
end
