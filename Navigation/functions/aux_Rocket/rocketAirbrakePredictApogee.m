function [apogeeAGL,valid] = rocketAirbrakePredictApogee(hAGL,vUp,Rocket,Weather,CdLookup,Airbrake)
%ROCKETAIRBRAKEPREDICTAPOGEE Simple vertical, unbraked coast forecast.
% Uses dry mass, local density and level-zero CSV Cd. Ignores attitude,
% horizontal velocity, wind and normal forces; intended for initial testing.
% Midpoint integration stops at the vertical zero crossing, not a flight timer.
apogeeAGL=hAGL;
valid=false;
if ~all(isfinite([hAGL;vUp])) || hAGL<0, return; end
if vUp<=0, valid=true; return; end
height=hAGL;
velocity=vUp;
dt=Airbrake.PredictionStep;
for k=1:Airbrake.PredictionMaxSteps
    a=acceleration(height,velocity,Rocket,Weather,CdLookup);
    midVelocity=velocity+0.5*dt*a;
    midHeight=height+0.5*dt*velocity;
    nextVelocity=velocity+dt*acceleration(midHeight,midVelocity,Rocket,Weather,CdLookup);
    nextHeight=height+dt*midVelocity;
    if nextVelocity<=0
        fraction=velocity/(velocity-nextVelocity);
        apogeeAGL=height+fraction*(nextHeight-height);
        valid=isfinite(apogeeAGL);
        return
    end
    height=nextHeight;
    velocity=nextVelocity;
end
end

function a=acceleration(h,v,Rocket,Weather,CdLookup)
[~,rho]=rocketWeatherConditions(Rocket.AltitudeMSL+h,Rocket,Weather,zeros(3,1));
cd=interp1(CdLookup.VelocityBreakpoints,CdLookup.Table(:,1), ...
    min(CdLookup.VelocityBreakpoints(end),max(CdLookup.VelocityBreakpoints(1),abs(v))), 'linear');
g=9.80665*(6371000/(6371000+Rocket.AltitudeMSL+h))^2;
a=-g-0.5*rho*cd*Rocket.ReferenceArea/Rocket.DryMass*v*abs(v);
end
