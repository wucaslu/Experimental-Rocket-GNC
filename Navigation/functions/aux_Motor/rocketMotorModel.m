function [thrust,motorMass,motorInertia,massRate,inertiaRate,remaining,Geometry] = rocketMotorModel(tau,StaticFire,typeCode,Properties)
%ROCKETMOTORMODEL Evaluate a prepared static-fire motor without file I/O.
% INPUTS
%   tau: finite scalar time since motor ignition [s]; negative is preburn.
%   StaticFire: numeric curve/profile structure from rocketMotorPrepare.
%   typeCode: 1 solid, 2 hybrid, 3 liquid, selected during initialization.
%   Properties: validated numeric motor masses, intrinsic diagonal inertias
%       about each component's own center, and axial body-reference positions.
% OUTPUTS
%   thrust: measured-curve axial thrust [N], zero outside the curve.
%   motorMass: dry motor plus remaining fuel/oxidizer mass [kg].
%   motorInertia: diagonal total motor inertia about its moving center
%       [3x1 kg*m^2]; exact zero is allowed for an omitted contribution.
%   massRate: time derivative of remaining motor mass [kg/s].
%   inertiaRate: derivative of diagonal motor inertia [3x1 kg*m^2/s].
%   remaining: actual [fuel;oxidizer] masses [2x1 kg], including residuals.
%   Geometry: numeric component geometry; array rows/columns follow
%       [dry;fuel;oxidizer]. Masses, MassRates and MassAccelerations are 3x1
%       [kg], [kg/s], [kg/s^2]. CentersZ, CenterRatesZ and
%       CenterAccelerationsZ are 3x1 axial body-reference coordinates [m],
%       [m/s], [m/s^2]. Inertias and InertiaRates are 3x3 intrinsic diagonal
%       vectors by component column [kg*m^2], [kg*m^2/s]. CenterOfMass,
%       CenterOfMassRate and CenterOfMassAcceleration are 3x1 body vectors
%       [m], [m/s], [m/s^2], with only the axial component nonzero.
% Channel centers interpolate with remaining fraction. Enabled solid BATES
% inertia follows the remaining mass's evolving bore radius and grain height;
% other channels retain endpoint interpolation.
% Parallel-axis terms place total motor inertia at the current motor center.
% Measured mass profiles interpolate linearly. Absent profiles use exact
% integrated piecewise-linear thrust with constant effective exhaust velocity.
% Hybrid/liquid impulse fallback assumes constant consumed oxidizer/fuel ratio.
% This is data-driven motor replay, not a grain/chamber combustion prediction.
% All state is supplied explicitly; there are no retained variables.
%#codegen

assert(isscalar(tau) && isreal(tau) && isfinite(tau), ...
    'RocketMotor:RuntimeTime','Time since ignition must be finite and scalar.');
assert(typeCode==1 || typeCode==2 || typeCode==3, ...
    'RocketMotor:RuntimeType','Runtime motor type must be 1, 2, or 3.');
fuelConsumed = Properties.FuelInitialMass-Properties.FuelResidualMass;
oxidizerConsumed = Properties.OxidizerInitialMass-Properties.OxidizerResidualMass;
thrust = 0;
fuelFraction = 1;
oxidizerFraction = 1;
fuelFractionRate = 0;
oxidizerFractionRate = 0;
fuelFractionAcceleration = 0;
oxidizerFractionAcceleration = 0;
count = numel(StaticFire.Time);
if tau>=StaticFire.Time(count)
    fuelFraction = 0;
    oxidizerFraction = 0;
elseif tau>=StaticFire.Time(1)
    % Binary search returns the interval whose left endpoint is <= tau.
    left = 1;
    right = count;
    while right-left>1
        middle = floor((left+right)/2);
        if tau<StaticFire.Time(middle)
            right = middle;
        else
            left = middle;
        end
    end
    interval = left;
    duration = StaticFire.Time(interval+1)-StaticFire.Time(interval);
    elapsed = tau-StaticFire.Time(interval);
    thrustSlope = (StaticFire.Thrust(interval+1)-StaticFire.Thrust(interval))/duration;
    thrust = StaticFire.Thrust(interval)+thrustSlope*elapsed;
    impulse = StaticFire.CumulativeImpulse(interval) + ...
        StaticFire.Thrust(interval)*elapsed+0.5*thrustSlope*elapsed*elapsed;
    impulseFraction = max(0,min(1,1-impulse/StaticFire.TotalImpulse));
    impulseFractionRate = -thrust/StaticFire.TotalImpulse;
    impulseFractionAcceleration = -thrustSlope/StaticFire.TotalImpulse;
    if StaticFire.HasFuelMassProfile>0.5
        fuelFractionRate = (StaticFire.FuelFraction(interval+1)- ...
            StaticFire.FuelFraction(interval))/duration;
        fuelFraction = StaticFire.FuelFraction(interval)+fuelFractionRate*elapsed;
    else
        fuelFraction = impulseFraction;
        fuelFractionRate = impulseFractionRate;
        fuelFractionAcceleration = impulseFractionAcceleration;
    end
    if StaticFire.HasOxidizerMassProfile>0.5
        oxidizerFractionRate = (StaticFire.OxidizerFraction(interval+1)- ...
            StaticFire.OxidizerFraction(interval))/duration;
        oxidizerFraction = StaticFire.OxidizerFraction(interval)+oxidizerFractionRate*elapsed;
    else
        oxidizerFraction = impulseFraction;
        oxidizerFractionRate = impulseFractionRate;
        oxidizerFractionAcceleration = impulseFractionAcceleration;
    end
end
remaining = [Properties.FuelResidualMass+fuelFraction*fuelConsumed; ...
    Properties.OxidizerResidualMass+oxidizerFraction*oxidizerConsumed];
Masses = [Properties.DryMass;remaining];
MassRates = [0;fuelFractionRate*fuelConsumed;oxidizerFractionRate*oxidizerConsumed];
MassAccelerations = [0;fuelFractionAcceleration*fuelConsumed; ...
    oxidizerFractionAcceleration*oxidizerConsumed];
motorMass = sum(Masses);
massRate = sum(MassRates);
fuelInertiaChange = Properties.FuelInitialInertia-Properties.FuelResidualInertia;
oxidizerInertiaChange = Properties.OxidizerInitialInertia-Properties.OxidizerResidualInertia;
Inertias = [Properties.DryInertia, ...
    Properties.FuelResidualInertia+fuelFraction*fuelInertiaChange, ...
    Properties.OxidizerResidualInertia+oxidizerFraction*oxidizerInertiaChange];
InertiaRates = [zeros(3,1),fuelFractionRate*fuelInertiaChange, ...
    oxidizerFractionRate*oxidizerInertiaChange];
if typeCode==1 && isfield(Properties,'GrainInertiaEnabled') && Properties.GrainInertiaEnabled>0.5
    [Inertias(:,2),InertiaRates(:,2)] = ...
        rocketMotorSolidGrainInertia(remaining(1),MassRates(2),Properties);
end
fuelCenterChange = Properties.FuelInitialCenterOfMassZ-Properties.FuelResidualCenterOfMassZ;
oxidizerCenterChange = Properties.OxidizerInitialCenterOfMassZ-Properties.OxidizerResidualCenterOfMassZ;
CentersZ = [Properties.DryCenterOfMassZ; ...
    Properties.FuelResidualCenterOfMassZ+fuelFraction*fuelCenterChange; ...
    Properties.OxidizerResidualCenterOfMassZ+oxidizerFraction*oxidizerCenterChange];
CenterRatesZ = [0;fuelFractionRate*fuelCenterChange; ...
    oxidizerFractionRate*oxidizerCenterChange];
CenterAccelerationsZ = [0;fuelFractionAcceleration*fuelCenterChange; ...
    oxidizerFractionAcceleration*oxidizerCenterChange];

centerZ = 0;
centerRateZ = 0;
centerAccelerationZ = 0;
if motorMass>0
    firstMoment = sum(Masses.*CentersZ);
    firstMomentRate = sum(MassRates.*CentersZ+Masses.*CenterRatesZ);
    firstMomentAcceleration = sum(MassAccelerations.*CentersZ + ...
        2*MassRates.*CenterRatesZ+Masses.*CenterAccelerationsZ);
    centerZ = firstMoment/motorMass;
    centerRateZ = (firstMomentRate-massRate*centerZ)/motorMass;
    centerAccelerationZ = (firstMomentAcceleration - ...
        sum(MassAccelerations)*centerZ-2*massRate*centerRateZ)/motorMass;
end
offsetsZ = CentersZ-centerZ;
offsetRatesZ = CenterRatesZ-centerRateZ;
parallelAxis = sum(Masses.*offsetsZ.*offsetsZ);
parallelAxisRate = sum(MassRates.*offsetsZ.*offsetsZ + ...
    2*Masses.*offsetsZ.*offsetRatesZ);
motorInertia = sum(Inertias,2)+[parallelAxis;parallelAxis;0];
inertiaRate = sum(InertiaRates,2)+[parallelAxisRate;parallelAxisRate;0];
Geometry = struct('Masses',Masses,'MassRates',MassRates, ...
    'MassAccelerations',MassAccelerations,'CentersZ',CentersZ, ...
    'CenterRatesZ',CenterRatesZ,'CenterAccelerationsZ',CenterAccelerationsZ, ...
    'Inertias',Inertias,'InertiaRates',InertiaRates, ...
    'CenterOfMass',[0;0;centerZ],'CenterOfMassRate',[0;0;centerRateZ], ...
    'CenterOfMassAcceleration',[0;0;centerAccelerationZ]);
end
