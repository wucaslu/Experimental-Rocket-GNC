function [vars,sample] = rocketMonteCarloSample(nominal,options,runId)
%ROCKETMONTECARLOSAMPLE Reproducible, physically consistent flight parameters.
% INPUTS: nominal initialized model variables, runner OPTIONS, positive RUNID.
% OUTPUTS: VARS supplies temporary SimulationInput values; SAMPLE records every
% draw and motor-derived factors. Private Threefry substreams avoid changing
% MATLAB's global rng. No persistent state or saved-model changes are used.
% Airbrake/CdLookup/MEKF calibration remain nominal. AxialDragScale represents
% unknown axial Cd*A error while ReferenceArea remains geometric; fin/body
% surface geometry is rebuilt separately. Sensor calibration/noise amplitudes
% remain fixed; only existing random seeds are optionally renewed.
stream=RandStream('Threefry','Seed',options.Seed);
stream.Substream=runId;
u=options.Uncertainty; b=options.TruncationSigma;
sample.RunId=runId;
sample.DryMassScale=positive(stream,u.DryMassRelative,b);
sample.DryInertiaScale=positive(stream,u.DryInertiaRelative,b);
sample.AirframeCenterOffsetM=normal(stream,u.AirframeCenterOfMassM,b);
sample.BodyRadiusScale=positive(stream,u.BodyRadiusRelative,b);
sample.FinDimensionScale=positive(stream,u.FinDimensionsRelative,b);
sample.FinCantOffsetRad=normal(stream,u.FinCantRad,b);
sample.FinLiftSlopeScale=positive(stream,u.FinLiftSlopeRelative,b);
sample.DragAreaScale=positive(stream,u.DragAreaRelative,b);
sample.MotorDryMassScale=positive(stream,u.MotorDryMassRelative,b);
sample.MotorInertiaScale=positive(stream,u.MotorInertiaRelative,b);
sample.MotorPositionOffsetM=normal(stream,u.MotorPositionM,b);
sample.MotorDryCenterOffsetM=normal(stream,u.MotorDryCenterOfMassM,b);
sample.GrainDensityScale=positive(stream,u.GrainDensityRelative,b);
sample.GrainDimensionScale=positive(stream,u.GrainDimensionsRelative,b);
sample.FuelMassScale=positive(stream,u.FuelMassRelative,b);
sample.OxidizerMassScale=positive(stream,u.OxidizerMassRelative,b);
sample.ExhaustVelocityScale=positive(stream,u.ExhaustVelocityRelative,b);
sample.BurnTimeScale=positive(stream,u.BurnTimeRelative,b);
sample.RailLengthScale=positive(stream,u.RailLengthRelative,b);
sample.LaunchInclinationOffsetRad=normal(stream,u.LaunchInclinationRad,b);
sample.LaunchHeadingOffsetRad=normal(stream,u.LaunchHeadingRad,b);
sample.LaunchRollOffsetRad=normal(stream,u.LaunchRollRad,b);
sample.LaunchAltitudeOffsetM=normal(stream,u.LaunchAltitudeM,b);
sample.IgnitionTimeOffsetS=normal(stream,u.IgnitionTimeS,b);
sample.DrogueCdSScale=positive(stream,u.CanopyCdSRelative,b);
sample.MainCdSScale=positive(stream,u.CanopyCdSRelative,b);
sample.DrogueInflationScale=positive(stream,u.CanopyInflationRelative,b);
sample.MainInflationScale=positive(stream,u.CanopyInflationRelative,b);
sample.DrogueLagOffsetS=normal(stream,u.RecoveryLagS,b);
sample.MainLagOffsetS=normal(stream,u.RecoveryLagS,b);
sample.MainTriggerOffsetM=normal(stream,u.MainTriggerAltitudeM,b);
sample.TemperatureOffsetK=normal(stream,u.TemperatureK,b);
sample.PressureScale=positive(stream,u.PressureRelative,b);
sample.LapseRateOffsetKpm=normal(stream,u.LapseRateKpm,b);
sample.RelativeHumidityOffset=normal(stream,u.RelativeHumidity,b);
sample.WindOffsetNED=normal(stream,u.WindOffsetNED,b);
sample.WindShearNED=normal(stream,u.WindShearNED,b);
sample.GustRmsNED=abs(normal(stream,u.AdditionalGustRmsNED,b));
sample.GustPhases=2*pi*rand(stream,size(nominal.Weather.GustPhases));
if all(u.AdditionalGustRmsNED==0),sample.GustPhases=nominal.Weather.GustPhases;end
sample.NavPositionErrorNEDM=normal(stream,u.NavPositionNEDM,b);
sample.NavVelocityErrorNEDMps=normal(stream,u.NavVelocityNEDMps,b);
sample.NavAttitudeErrorRad=normal(stream,u.NavAttitudeRad,b);
sample.NavGyroBiasErrorRadps=normal(stream,u.NavGyroBiasRadps,b);
sample.NavAccelBiasErrorMps2=normal(stream,u.NavAccelBiasMps2,b);
vars=nominal;
R=nominal.Rocket;
R.DryMass=R.DryMass*sample.DryMassScale;
R.DryInertia=R.DryInertia*sample.DryMassScale*sample.DryInertiaScale;
R.DryCenterOfMassZ=R.DryCenterOfMassZ+sample.AirframeCenterOffsetM;
R.BodyRadius=R.BodyRadius*sample.BodyRadiusScale;
R.ReferenceArea=pi*R.BodyRadius^2;
R.NoseBaseRadius=R.NoseBaseRadius*sample.BodyRadiusScale;
R.TailTopRadius=R.TailTopRadius*sample.BodyRadiusScale;
R.TailBottomRadius=R.TailBottomRadius*sample.BodyRadiusScale;
finNames={'FinRootChord','FinTipChord','FinSpan','FinSweepLength'};
for k=1:numel(finNames),R.(finNames{k})=R.(finNames{k})*sample.FinDimensionScale;end
R.FinCant=R.FinCant+sample.FinCantOffsetRad;
R.FinLiftSlope2D=R.FinLiftSlope2D*sample.FinLiftSlopeScale;
R.RailLength=R.RailLength*sample.RailLengthScale;
R.LaunchInclination=R.LaunchInclination+sample.LaunchInclinationOffsetRad;
assert(R.LaunchInclination>0 && R.LaunchInclination<=pi/2, ...
    'RocketMC:Inclination','Sampled rail inclination must be in (0,pi/2].');
R.LaunchHeading=mod(R.LaunchHeading+sample.LaunchHeadingOffsetRad,2*pi);
R.LaunchRoll=R.LaunchRoll+sample.LaunchRollOffsetRad;
R.AltitudeMSL=R.AltitudeMSL+sample.LaunchAltitudeOffsetM;
R.IgnitionTime=R.IgnitionTime+sample.IgnitionTimeOffsetS;
assert(R.IgnitionTime>=0,'RocketMC:Ignition','Sampled ignition cannot precede model time zero.');
R.DrogueCdS=R.DrogueCdS*sample.DrogueCdSScale;
R.MainCdS=R.MainCdS*sample.MainCdSScale;
R.DrogueInflationTime=R.DrogueInflationTime*sample.DrogueInflationScale;
R.MainInflationTime=R.MainInflationTime*sample.MainInflationScale;
R.DrogueLag=max(0,R.DrogueLag+sample.DrogueLagOffsetS);
R.MainLag=max(0,R.MainLag+sample.MainLagOffsetS);
R.MainAltitudeAGL=max(0,R.MainAltitudeAGL+sample.MainTriggerOffsetM);
[R,motorInfo]=rocketMonteCarloMotor(nominal.Rocket,R,sample);
R.AxialDragScale=sample.DragAreaScale;
if isfield(nominal.Rocket,'AxialDragScale')
    R.AxialDragScale=nominal.Rocket.AxialDragScale*sample.DragAreaScale;
end
rocketPlantValidateConfig(R);
vars.Rocket=R;
vars.Weather=rocketMonteCarloWeather(nominal.Weather,nominal.Rocket,R,sample);
vars.Rocket.WindAltitude=vars.Weather.WindAltitudeAGL;
vars.Rocket.WindNED=vars.Weather.WindNED;
vars.RocketInitialState=rocketPlantInitialize(vars.Rocket);
vars.LLA0=[R.Latitude;R.Longitude;R.AltitudeMSL];
% Rail uncertainty is true geometry; onboard initial attitude retains its
% nominal surveyed value plus a separate initial-estimate error.
vars.q0=attitudeError(nominal.q0,sample.NavAttitudeErrorRad);
vars.p0=nominal.p0+sample.NavPositionErrorNEDM;
vars.v0=nominal.v0+sample.NavVelocityErrorNEDMps;
vars.bg0=nominal.bg0+sample.NavGyroBiasErrorRadps;
vars.ba0=nominal.ba0+sample.NavAccelBiasErrorMps2;
vars.Airbrake.LaunchAltitudeMSL=R.AltitudeMSL; % known surveyed site per case
[vars.pressure0,~]=ISA_model(0,vars.LLA0(3)); % nominal startup model, not true weather
if sample.LaunchAltitudeOffsetM~=0 && vars.Sensor.GPS.SourceAltitudeIsMSL
    vars.Sensor.GPS.GeoidSeparation=geoidheight(rad2deg(R.Latitude), ...
        mod(rad2deg(R.Longitude),360),'EGM96');
end
if options.RandomizeNoiseSeeds
    seeds=zeros(40,1);
    for seedIndex=1:40
        candidate=randi(stream,2147483646);
        while any(seeds(1:seedIndex-1)==candidate)
            candidate=randi(stream,2147483646);
        end
        seeds(seedIndex)=candidate;
    end
    sample.NoiseSeeds=seeds;
    names={'Gyro','Acc','Mag'}; kinds={'Brown','White','Pink'}; index=0;
    for k=1:numel(names)
        for j=1:numel(kinds)
            vars.Sensor.(names{k}).Noise.(kinds{j}).Seed=seeds(index+(1:3));
            index=index+3;
        end
    end
    for j=1:numel(kinds)
        index=index+1;vars.Sensor.Baro.Noise.(kinds{j}).Seed=seeds(index);
    end
    vars.Sensor.GPS.Seed=seeds(31:40);
else
    sample.NoiseSeeds=zeros(0,1);
end
vars.GNSSConfig=vars.Sensor.GPS;
vars.GNSSInitialState=gnssPositionInitialize(vars.LLA0,vars.GNSSConfig);
sample.Motor=motorInfo;
sample.PropellantGeometryEnabled=R.Motor.Properties.GrainInertiaEnabled>.5;
sample.FuelMassScaleActive=~sample.PropellantGeometryEnabled;
sample.OxidizerMassScaleActive=R.Motor.Properties.OxidizerInitialMass>0;
end

function value=normal(stream,sigma,bound)
% Inverse-CDF draw needs no Statistics Toolbox and no rejection loop.
lower=.5*(1+erf(-bound/sqrt(2)));upper=1-lower;
z=sqrt(2)*erfinv(2*(lower+(upper-lower)*rand(stream,size(sigma)))-1);
value=sigma.*z;
end

function value=positive(stream,relativeSigma,bound)
logSigma=sqrt(log1p(relativeSigma.^2));
value=exp(-.5*logSigma.^2+normal(stream,logSigma,bound));
end

function q=attitudeError(nominal,error)
angle=norm(error);
delta=[zeros(3,1);1];
if angle>0,delta=[error*sin(angle/2)/angle;cos(angle/2)];end
nominal=nominal(:)/norm(nominal);
q=[delta(4)*nominal(1:3)+nominal(4)*delta(1:3)+cross(delta(1:3),nominal(1:3)); ...
    delta(4)*nominal(4)-dot(delta(1:3),nominal(1:3))];
q=q/norm(q);
end
