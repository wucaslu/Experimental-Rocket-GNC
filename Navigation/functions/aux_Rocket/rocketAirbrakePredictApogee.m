function [apogeeAGL,valid] = rocketAirbrakePredictApogee(hAGL,vUp,CdLookup,Airbrake)
%ROCKETAIRBRAKEPREDICTAPOGEE Nominal vertical, unbraked coast forecast.
% Inputs: hAGL [m] and vUp [m/s] are Navigation-estimated height above launch
% and upward speed; CdLookup is the onboard airspeed/Mach deployment table;
% Airbrake supplies nominal dry mass, reference area, surveyed launch MSL
% altitude and numerical forecast settings, independently of the truth plant.
% Outputs: apogeeAGL [m] is the predicted peak above launch; valid is false
% for invalid initial estimates or an unfinished coast integration.
% Uses dry standard-atmosphere density and level-zero absolute CSV Cd,
% including Mode=2 total Cd queried at estimated speed / ISA sound speed. For
% a Mode=1 added-airbrake source, Airbrake.NominalMach/NominalCdCoast supply the
% independent nominal coast curve; zero deployment adds no drag. Wind, actual
% weather, horizontal velocity, attitude and normal forces are unobserved.
% Midpoint integration stops at the vertical zero crossing, not a flight timer.
apogeeAGL=hAGL;
valid=false;
if ~all(isfinite([hAGL;vUp])) || hAGL<0, return; end
if vUp<=0, valid=true; return; end
rocketPlantValidateCdLookup(CdLookup);
height=hAGL;
velocity=vUp;
dt=Airbrake.PredictionStep;
for k=1:Airbrake.PredictionMaxSteps
    a=acceleration(height,velocity,CdLookup,Airbrake);
    midVelocity=velocity+0.5*dt*a;
    midHeight=height+0.5*dt*velocity;
    nextVelocity=velocity+dt*acceleration(midHeight,midVelocity,CdLookup,Airbrake);
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

function a=acceleration(h,v,CdLookup,Airbrake)
altitudeMSL=Airbrake.LaunchAltitudeMSL+h;
[rho,~,soundSpeed]=rocketPlantAtmosphere(altitudeMSL);
if isfield(CdLookup,'Mode') && isfield(CdLookup,'Samples')
    % Field guards also let fixed Mode=2 structs compile without Samples.
    assert(isfield(Airbrake,'NominalMach') && isfield(Airbrake,'NominalCdCoast'), ...
        'Airbrake:MissingNominalDrag', ...
        'Added-airbrake data requires independent NominalMach/NominalCdCoast for coast prediction.');
    cd=rocketPlantInterpolate(Airbrake.NominalMach,Airbrake.NominalCdCoast,abs(v)/soundSpeed);
else
    % Absolute velocity/Mach grids provide total Cd even when level is zero.
    cd=rocketPlantCdLookup(abs(v),0,CdLookup,soundSpeed);
end
g=9.80665*(6371000/(6371000+altitudeMSL))^2;
a=-g-0.5*rho*cd*Airbrake.ReferenceArea/Airbrake.DryMass*v*abs(v);
end
