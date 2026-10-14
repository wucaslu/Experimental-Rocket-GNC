function [Rocket,info] = rocketMonteCarloMotor(nominalRocket,Rocket,sample)
%ROCKETMONTECARLOMOTOR Rebuild one physically consistent sampled motor/rocket.
%   [Rocket,info] = ROCKETMONTECARLOMOTOR(nominalRocket,Rocket,sample)
%   prepares the runtime motor and dependent rocket geometry for one Monte
%   Carlo realization. It performs no model edits, simulation or file I/O.
%
% INPUTS
%   nominalRocket: validated nominal runtime Rocket structure, including its
%       prepared Motor.StaticFire, numeric Motor.Type and Motor.Properties.
%   Rocket: sampled airframe configuration. The caller has already varied
%       DryMass [kg], intrinsic DryInertia [3x1 kg*m^2], DryCenterOfMassZ [m],
%       aerodynamic geometry, fin cant and other desired rocket properties.
%       Positions initially use the nominal body-fixed origin O, +Z noseward.
%   sample: scalar structure with these finite real scalar fields:
%       MotorDryMassScale: positive multiplier on dry motor mass.
%       MotorInertiaScale: positive multiplier on its specific inertia;
%           actual dry inertia scales by both mass and this multiplier.
%       MotorPositionOffsetM: signed axial installation offset [m], applied
%           to all motor component centers and the rocket's nozzle station.
%       MotorDryCenterOffsetM: extra signed dry-hardware CM offset [m].
%       GrainDensityScale: positive solid-grain density multiplier.
%       GrainDimensionScale: positive common multiplier on grain outer/inner
%           radii, initial height and separation. Enabled BATES propellant
%           mass follows volume*density; its residual fraction is preserved.
%       FuelMassScale/OxidizerMassScale: positive mass/intrinsic-inertia
%           multipliers for non-grain fuel/oxidizer endpoint models. These
%           factors are unused for an enabled solid BATES geometry.
%       ExhaustVelocityScale: positive effective exhaust velocity multiplier.
%       BurnTimeScale: positive multiplier on static-fire time since ignition.
%       Grain factors are unused when grain inertia is disabled.
%
% OUTPUTS
%   Rocket: configuration with a newly prepared numeric motor, rebuilt aero
%       coefficients and validated parameters. Measured mass-fraction shapes
%       are preserved and impulse caches are recomputed from the new curve.
%       In RocketPy-compatible momentum mode 1, all fixed body-Z stations are
%       shifted together to put O at the sampled combined dry CM. Intrinsic
%       component inertias remain about their own centers. Mode 0 preserves
%       the caller's fixed body reference. Thrust remains axial in mode 1.
%   info: scalar structure with ImpulseScale and ThrustScale [-],
%       BurnTimeScale [-], ConsumedMassKg [kg], EffectiveExhaustVelocityMps
%       [m/s] (=total impulse/consumed mass), and BodyReferenceShiftM [m], the
%       positive-Z displacement subtracted from stations when recentering.
%
% Total impulse scales with consumed mass and effective exhaust velocity.
% Thrust amplitude is impulse scale divided by burn-time scale. This retains
% the nominal thrust/mass-profile shapes; it is a static-fire replay model,
% not a combustion prediction. The nozzle radius inferred from nominal axial
% gyration is held fixed while its origin-dependent gyration is recomputed.
% An identity sample with an unchanged Rocket returns the nominal numeric
% configuration exactly. There are no persistent or random variables here.

scaleFields = {'MotorDryMassScale','MotorInertiaScale', ...
    'GrainDensityScale','GrainDimensionScale','FuelMassScale', ...
    'OxidizerMassScale','ExhaustVelocityScale','BurnTimeScale'};
offsetFields = {'MotorPositionOffsetM','MotorDryCenterOffsetM'};
assert(isstruct(sample) && isscalar(sample), ...
    'RocketMonteCarlo:MotorSampleType','sample must be a scalar structure.');
for index = 1:numel(scaleFields)
    name = scaleFields{index};
    assert(isfield(sample,name) && finiteScalar(sample.(name)) && ...
        sample.(name)>0, 'RocketMonteCarlo:MotorScale', ...
        'sample.%s must be a finite positive real scalar.',name);
end
for index = 1:numel(offsetFields)
    name = offsetFields{index};
    assert(isfield(sample,name) && finiteScalar(sample.(name)), ...
        'RocketMonteCarlo:MotorOffset', ...
        'sample.%s must be a finite real scalar in meters.',name);
end

nominalMotor = nominalRocket.Motor;
nominalProperties = nominalMotor.Properties;
nominalFire = nominalMotor.StaticFire;
nominalConsumedMass = nominalProperties.FuelInitialMass - ...
    nominalProperties.FuelResidualMass + ...
    nominalProperties.OxidizerInitialMass - nominalProperties.OxidizerResidualMass;
assert(isfinite(nominalConsumedMass) && nominalConsumedMass>0, ...
    'RocketMonteCarlo:MotorConsumption', ...
    'A positive nominal consumed mass is required for coherent impulse scaling.');

identitySample = true;
for index = 1:numel(scaleFields)
    identitySample = identitySample && sample.(scaleFields{index})==1;
end
for index = 1:numel(offsetFields)
    identitySample = identitySample && sample.(offsetFields{index})==0;
end
if identitySample && isequaln(Rocket,nominalRocket)
    rocketPlantValidateConfig(Rocket);
    info = sampleInfo(1,1,1,nominalConsumedMass, ...
        nominalFire.TotalImpulse/nominalConsumedMass,0);
    return
end

properties = nominalProperties;
properties.DryMass = nominalProperties.DryMass*sample.MotorDryMassScale;
properties.DryInertia = nominalProperties.DryInertia * ...
    (sample.MotorDryMassScale*sample.MotorInertiaScale);
motorCenterFields = {'DryCenterOfMassZ','FuelInitialCenterOfMassZ', ...
    'FuelResidualCenterOfMassZ','OxidizerInitialCenterOfMassZ', ...
    'OxidizerResidualCenterOfMassZ'};
for index = 1:numel(motorCenterFields)
    name = motorCenterFields{index};
    properties.(name) = nominalProperties.(name)+sample.MotorPositionOffsetM;
end
properties.DryCenterOfMassZ = properties.DryCenterOfMassZ + ...
    sample.MotorDryCenterOffsetM;
Rocket.ThrustOffset(3) = Rocket.ThrustOffset(3)+sample.MotorPositionOffsetM;

grainEnabled = isfield(properties,'GrainInertiaEnabled') && ...
    properties.GrainInertiaEnabled>0.5;
if grainEnabled
    dimensionFields = {'GrainOuterRadius','GrainInitialInnerRadius', ...
        'GrainInitialHeight','GrainSeparation'};
    for index = 1:numel(dimensionFields)
        name = dimensionFields{index};
        properties.(name) = nominalProperties.(name)*sample.GrainDimensionScale;
    end
    properties.GrainDensity = nominalProperties.GrainDensity*sample.GrainDensityScale;
    if sample.GrainDimensionScale==1 && sample.GrainDensityScale==1
        % Retain the already validated mass exactly when geometry is unchanged.
        properties.FuelInitialMass = nominalProperties.FuelInitialMass;
    else
        properties.FuelInitialMass = properties.GrainCount*properties.GrainDensity*pi * ...
            (properties.GrainOuterRadius^2-properties.GrainInitialInnerRadius^2) * ...
            properties.GrainInitialHeight;
    end
    assert(nominalProperties.FuelInitialMass>0, ...
        'RocketMonteCarlo:GrainMass','Enabled nominal grains require positive initial mass.');
    fuelScale = properties.FuelInitialMass/nominalProperties.FuelInitialMass;
    properties.FuelResidualMass = nominalProperties.FuelResidualMass*fuelScale;
    % rocketMotorPrepare derives both fuel-inertia endpoints from geometry.
else
    properties.FuelInitialMass = nominalProperties.FuelInitialMass*sample.FuelMassScale;
    properties.FuelResidualMass = nominalProperties.FuelResidualMass*sample.FuelMassScale;
    properties.FuelInitialInertia = nominalProperties.FuelInitialInertia*sample.FuelMassScale;
    properties.FuelResidualInertia = nominalProperties.FuelResidualInertia*sample.FuelMassScale;
    properties.OxidizerInitialMass = nominalProperties.OxidizerInitialMass*sample.OxidizerMassScale;
    properties.OxidizerResidualMass = nominalProperties.OxidizerResidualMass*sample.OxidizerMassScale;
    properties.OxidizerInitialInertia = nominalProperties.OxidizerInitialInertia*sample.OxidizerMassScale;
    properties.OxidizerResidualInertia = nominalProperties.OxidizerResidualInertia*sample.OxidizerMassScale;
end

consumedMass = properties.FuelInitialMass-properties.FuelResidualMass + ...
    properties.OxidizerInitialMass-properties.OxidizerResidualMass;
assert(isfinite(consumedMass) && consumedMass>0, ...
    'RocketMonteCarlo:MotorConsumption','Sampled consumed mass must be positive and finite.');
impulseScale = (consumedMass/nominalConsumedMass)*sample.ExhaustVelocityScale;
thrustScale = impulseScale/sample.BurnTimeScale;
assert(isfinite(impulseScale) && impulseScale>0 && ...
    isfinite(thrustScale) && thrustScale>0, ...
    'RocketMonteCarlo:MotorImpulse','Sampled impulse and thrust scales must be positive and finite.');

referenceShift = 0;
if isfield(Rocket,'MomentumModel') && Rocket.MomentumModel==1
    hardwareMass = Rocket.DryMass+properties.DryMass;
    assert(isfinite(hardwareMass) && hardwareMass>0, ...
        'RocketMonteCarlo:DryHardwareMass','Combined dry hardware mass must be positive and finite.');
    center = (Rocket.DryMass*Rocket.DryCenterOfMassZ + ...
        properties.DryMass*properties.DryCenterOfMassZ)/hardwareMass;
    % Do not move a nominally centered reference for roundoff alone.
    centerScale = max([1;abs(Rocket.DryCenterOfMassZ);abs(properties.DryCenterOfMassZ)]);
    if abs(center)>16*eps(centerScale)
        referenceShift = center;
        Rocket.DryCenterOfMassZ = Rocket.DryCenterOfMassZ-referenceShift;
        for index = 1:numel(motorCenterFields)
            name = motorCenterFields{index};
            properties.(name) = properties.(name)-referenceShift;
        end
        rocketStationFields = {'NoseTipZ','FinLeadingEdgeZ','TailTopZ'};
        for index = 1:numel(rocketStationFields)
            name = rocketStationFields{index};
            Rocket.(name) = Rocket.(name)-referenceShift;
        end
        rocketVectorFields = {'ThrustOffset','IMUOffset','DrogueAttachment','MainAttachment'};
        for index = 1:numel(rocketVectorFields)
            name = rocketVectorFields{index};
            Rocket.(name)(3) = Rocket.(name)(3)-referenceShift;
        end
    end
end

source = table(nominalFire.Time*sample.BurnTimeScale, ...
    nominalFire.Thrust*thrustScale,'VariableNames',{'time_s','thrust_N'});
if nominalFire.HasFuelMassProfile==1
    source.fuel_mass_kg = properties.FuelResidualMass + ...
        nominalFire.FuelFraction*(properties.FuelInitialMass-properties.FuelResidualMass);
    source.fuel_mass_kg([1 end]) = [properties.FuelInitialMass;properties.FuelResidualMass];
end
if nominalFire.HasOxidizerMassProfile==1
    source.oxidizer_mass_kg = properties.OxidizerResidualMass + ...
        nominalFire.OxidizerFraction*(properties.OxidizerInitialMass-properties.OxidizerResidualMass);
    source.oxidizer_mass_kg([1 end]) = ...
        [properties.OxidizerInitialMass;properties.OxidizerResidualMass];
end
importOptions=struct();
if isfield(nominalFire,'AbruptEndpoints')
    assert(isa(nominalFire.AbruptEndpoints,'double') && ...
        isscalar(nominalFire.AbruptEndpoints) && nominalFire.AbruptEndpoints==1, ...
        'RocketMonteCarlo:MotorEndpoints','AbruptEndpoints must be the explicit numeric flag 1.');
    importOptions.EndpointMode='abrupt';
end
[staticFire,typeCode,properties] = rocketMotorPrepare(source,nominalMotor.Type,properties,importOptions);
Rocket.Motor = struct('StaticFire',staticFire,'Type',typeCode,'Properties',properties);

nozzleRadiusSquared = 2*nominalRocket.NozzleGyration(3);
nozzleDistanceSquared = Rocket.ThrustOffset(3)^2;
Rocket.NozzleGyration = [(nozzleRadiusSquared+nozzleDistanceSquared)/4; ...
    (nozzleRadiusSquared+nozzleDistanceSquared)/4;nozzleRadiusSquared/2];
Rocket = rocketPlantBuildAero(Rocket);
rocketPlantValidateConfig(Rocket);
info = sampleInfo(impulseScale,thrustScale,sample.BurnTimeScale, ...
    consumedMass,staticFire.TotalImpulse/consumedMass,referenceShift);
end

function valid = finiteScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end

function info = sampleInfo(impulseScale,thrustScale,burnTimeScale,mass,velocity,shift)
info = struct('ImpulseScale',impulseScale,'ThrustScale',thrustScale, ...
    'BurnTimeScale',burnTimeScale,'ConsumedMassKg',mass, ...
    'EffectiveExhaustVelocityMps',velocity,'BodyReferenceShiftM',shift);
end
