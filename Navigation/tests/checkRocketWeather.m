function report = checkRocketWeather(outputDirectory)
%CHECKROCKETWEATHER Independent thermodynamics, wind and disturbance checks.
% Tests the explicit weather interfaces and a short deterministic dynamics
% case. Does not open, edit or simulate a Simulink model, or read a CSV.
% Hydrostatic references use geometric-to-geopotential altitude and the
% closed-form isothermal ideal-gas solution. Moist density/sound are checked
% against an independent partial-pressure mixture calculation; its vapor
% pressure approximation is from the National Weather Service:
% https://www.weather.gov/media/epz/wxcalc/vaporPressure.pdf
if nargin<1
    outputDirectory='';
end
navigationDirectory=fileparts(fileparts(mfilename('fullpath')));
previousPath=path;
pathCleanup=onCleanup(@() path(previousPath)); %#ok<NASGU>
addpath(fullfile(navigationDirectory,'functions','aux_Rocket'));
[Rocket,Weather]=loadSettings(navigationDirectory);
names={'neutralLegacyConditions','stationAnchorAndMoistAir', ...
    'dryIsothermalHydrostatics','profileHydrostaticGradient', ...
    'meanWindAndHeldGust','fourierGustAndSmoothRamp','finiteDiscreteGust', ...
    'loadsPressureAndCPWind','shortDynamicsAndSampleHold', ...
    'invalidConfiguration','explicitWeatherSource'};
checks={@() checkNeutral(Rocket,Weather),@() checkStation(Rocket,Weather), ...
    @() checkIsothermal(Rocket,Weather),@() checkHydrostatic(Rocket,Weather), ...
    @() checkMeanWind(Rocket,Weather),@() checkFourier(Weather), ...
    @() checkPulse(Weather),@() checkLoads(Rocket,Weather), ...
    @() checkDynamics(Rocket,Weather),@() checkInvalid(Rocket,Weather), ...
    @() checkSource(navigationDirectory)};
results=repmat(struct('Name','','Passed',false,'ElapsedSeconds',0, ...
    'Metrics',struct(),'Message',''),numel(checks),1);
for k=1:numel(checks)
    results(k).Name=names{k};
    started=tic;
    try
        results(k).Metrics=checks{k}();
        results(k).Passed=true;
        fprintf('PASS %s\n',names{k});
    catch exception
        results(k).Message=exception.message;
        fprintf('FAIL %s: %s\n',names{k},exception.message);
    end
    results(k).ElapsedSeconds=toc(started);
end
report=struct('Passed',all([results.Passed]), ...
    'CreatedUTC',char(datetime('now','TimeZone','UTC')),'Checks',results);
if ~isempty(outputDirectory)
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    filename=fullfile(outputDirectory,'rocket-weather-checks.json');
    file=fopen(filename,'w');
    assert(file>=0,'RocketWeatherCheck:ReportFile','Cannot write %s.',filename);
    fileCleanup=onCleanup(@() fclose(file)); %#ok<NASGU>
    fprintf(file,'%s\n',jsonencode(report,PrettyPrint=true));
end
assert(report.Passed,'RocketWeatherCheck:Failed','Weather checks failed: %s.', ...
    strjoin({results(~[results.Passed]).Name},', '));
end

function [Rocket,Weather]=loadSettings(navigationDirectory)
run(fullfile(navigationDirectory,'settings','rocketPlantSettings.m'));
run(fullfile(navigationDirectory,'settings','weatherSettings.m'));
end

function W=neutralWeather(W,R)
W.UseProfile=0;
W.WindAltitudeAGL=R.WindAltitude;
W.WindNED=R.WindNED;
W.GustEnabled=0;
W.GustRmsNED=zeros(3,1);
W.DiscreteGustPeakNED=zeros(3,1);
end

function W=profileWeather(W,R)
W=neutralWeather(W,R);
W.UseProfile=1;
W.StationAltitudeMSL=R.AltitudeMSL;
W.StationTemperatureK=293.15;
W.StationPressurePa=90000;
W.StationRelativeHumidity=0.6;
W.LapseRateOffset=0;
W.HumidityScaleHeight=3000;
W.ProfileMinAltitudeMSL=0;
W.ProfileMaxAltitudeMSL=10000;
W.ProfileStepAltitude=25;
W=rocketWeatherBuildProfile(W);
end

function x=freeState(R)
x=rocketPlantInitialize(R);
x(1:3)=[0;0;-1000];
x(4:6)=[0;0;100];
x(7:10)=[0;0;0;1];
x(11:13)=zeros(3,1);
x(14)=2;
end

function H=geopotential(altitude)
radius=6356766;
H=radius*altitude./(radius+altitude);
end

function metrics=checkNeutral(R,W)
W=neutralWeather(W,R);
heights=[-100;0;R.AltitudeMSL;5000;11000;20000];
for k=1:numel(heights)
    h=heights(k);
    [rho0,p0,c0,T0]=rocketPlantAtmosphere(h);
    expectedWind=zeros(3,1);
    for j=1:3
        expectedWind(j)=interp1(R.WindAltitude,R.WindNED(:,j), ...
            min(max(h-R.AltitudeMSL,R.WindAltitude(1)),R.WindAltitude(end)),'linear');
    end
    [emptyWind,rhoEmpty,pEmpty,cEmpty,TEmpty,rhEmpty]= ...
        rocketWeatherConditions(h,R,[],zeros(3,1));
    [wind,rho,p,c,T,rh]=rocketWeatherConditions(h,R,W,zeros(3,1));
    assert(isequal([rho;p;c;T;rh],[rho0;p0;c0;T0;0]) && ...
        isequal([rhoEmpty;pEmpty;cEmpty;TEmpty;rhEmpty],[rho0;p0;c0;T0;0]) && ...
        norm(wind-expectedWind)<1e-12 && isequal(wind,emptyWind), ...
        'RocketWeatherCheck:Neutral','Omitted or disabled profile changes legacy ISA or inherited wind.');
end
assert(isequal(rocketWeatherGust(123,[]),zeros(3,1)) && ...
    isequal(rocketWeatherGust(123,W),zeros(3,1)), ...
    'RocketWeatherCheck:ZeroGust','Empty/default-disabled weather produces a gust.');
metrics=struct('AltitudesMSLCheckedM',heights,'ISAExactlyPreserved',true);
end

function metrics=checkStation(R,W)
W=profileWeather(W,R);
[~,rho,p,c,T,rh]=rocketWeatherConditions(R.AltitudeMSL,R,W,zeros(3,1));
assert(abs(T-W.StationTemperatureK)<1e-9 && ...
    abs(p-W.StationPressurePa)<1e-5 && ...
    abs(rh-W.StationRelativeHumidity)<1e-10, ...
    'RocketWeatherCheck:StationAnchor','Measured station temperature, pressure or fractional RH is not anchored exactly.');
% Independent NWS saturation approximation: 6.11 hPa, converted to Pa.
temperatureC=T-273.15;
saturationPa=611*10^(7.5*temperatureC/(237.3+temperatureC));
vaporPa=rh*saturationPa;
rhoDryPart=(p-vaporPa)/(287.05287*T);
rhoVaporPart=vaporPa/(461.5*T);
expectedRho=rhoDryPart+rhoVaporPart;
vaporMassFraction=rhoVaporPart/expectedRho;
mixtureR=p/(expectedRho*T);
mixtureCp=(1-vaporMassFraction)*(3.5*287.05287)+vaporMassFraction*1850;
expectedSound=sqrt(mixtureCp/(mixtureCp-mixtureR)*mixtureR*T);
dryRho=p/(287.05287*T);
drySound=sqrt(1.4*287.05287*T);
assert(abs(rho-expectedRho)/expectedRho<5e-4 && rho<dryRho && rho>0.9*dryRho && ...
    abs(c-expectedSound)/expectedSound<5e-4 && c>drySound && c<1.02*drySound && ...
    rh>0 && rh<=1 && p>50000 && p<110000 && T>250 && T<330, ...
    'RocketWeatherCheck:MoistUnits','Moist density/sound or temperature, pressure and RH units are incorrect.');
metrics=struct('TemperatureK',T,'PressurePa',p,'RelativeHumidityFraction',rh, ...
    'DensityKgpm3',rho,'NWSMixtureDensityOracleKgpm3',expectedRho, ...
    'SoundSpeedMps',c,'MixtureSoundOracleMps',expectedSound, ...
    'DryDensityKgpm3',dryRho,'DrySoundMps',drySound);
end

function metrics=checkIsothermal(R,W)
W=profileWeather(W,R);
W.StationRelativeHumidity=0;
W.LapseRateOffset=0.0065; % Cancels ISA geopotential troposphere lapse.
W=rocketWeatherBuildProfile(W);
heights=[W.StationAltitudeMSL-137;W.StationAltitudeMSL; ...
    W.StationAltitudeMSL+713;W.StationAltitudeMSL+1803;5000;9000];
maxPressureRelativeError=0;
maxTemperatureError=0;
for k=1:numel(heights)
    [~,rho,p,c,T,rh]=rocketWeatherConditions(heights(k),R,W,zeros(3,1));
    expectedP=W.StationPressurePa*exp(-9.80665* ...
        (geopotential(heights(k))-geopotential(W.StationAltitudeMSL))/ ...
        (287.05287*W.StationTemperatureK));
    maxPressureRelativeError=max(maxPressureRelativeError,abs(p-expectedP)/expectedP);
    maxTemperatureError=max(maxTemperatureError,abs(T-W.StationTemperatureK));
    assert(abs(T-W.StationTemperatureK)<1e-8 && abs(p-expectedP)/expectedP<2e-5 && ...
        abs(rho-p/(287.05287*T))<1e-10 && ...
        abs(c-sqrt(1.4*287.05287*T))<1e-8 && rh==0, ...
        'RocketWeatherCheck:Isothermal','Dry isothermal profile disagrees with hydrostatic ideal-gas solution.');
end
metrics=struct('MaximumPressureRelativeError',maxPressureRelativeError, ...
    'MaximumTemperatureErrorK',maxTemperatureError,'AltitudesMSLCheckedM',heights);
end

function metrics=checkHydrostatic(R,W)
W=profileWeather(W,R);
assert(all(diff(W.ProfileAltitudeMSL)>0) && all(diff(W.ProfilePressurePa)<0) && ...
    all(W.ProfilePressurePa>0) && ...
    all(W.ProfileRelativeHumidity>=0 & W.ProfileRelativeHumidity<=1), ...
    'RocketWeatherCheck:ProfilePhysical','Dense profile heights, pressure or humidity are unphysical.');
heights=[1500;3000;5000;8000];
delta=W.ProfileStepAltitude/2;
relativeErrors=zeros(size(heights));
for k=1:numel(heights)
    h=heights(k);
    [~,rho,~,~,~,rh]=rocketWeatherConditions(h,R,W,zeros(3,1));
    [~,~,pMinus]=rocketWeatherConditions(h-delta,R,W,zeros(3,1));
    [~,~,pPlus]=rocketWeatherConditions(h+delta,R,W,zeros(3,1));
    gradient=(pPlus-pMinus)/(geopotential(h+delta)-geopotential(h-delta));
    expected=-9.80665*rho;
    relativeErrors(k)=abs(gradient-expected)/abs(expected);
    assert(relativeErrors(k)<0.003 && rh>=0 && rh<=1, ...
        'RocketWeatherCheck:HydrostaticGradient','Profile violates dP/dH=-g*rho beyond dense-grid interpolation tolerance.');
end
metrics=struct('AltitudeMSLM',heights,'HydrostaticGradientRelativeErrors',relativeErrors, ...
    'GridStepM',W.ProfileStepAltitude);
end

function metrics=checkMeanWind(R,W)
W=neutralWeather(W,R);
W.WindAltitudeAGL=[0;1000;2000];
W.WindNED=[0 0 0;10 -4 2;20 -8 4];
heldGust=[-1;0.5;-0.2];
heightsAGL=[-100;0;500;1500;2500];
expected=[0 0 0;0 0 0;5 -2 1;15 -6 3;20 -8 4];
for k=1:numel(heightsAGL)
    h=R.AltitudeMSL+heightsAGL(k);
    wind=rocketWeatherWind(h,R,W,heldGust);
    [conditionsWind,~,~,~,~,~]=rocketWeatherConditions(h,R,W,heldGust);
    assert(norm(wind-expected(k,:).'-heldGust)<1e-12 && isequal(wind,conditionsWind), ...
        'RocketWeatherCheck:WindNED','Mean wind interpolation, AGL datum, Down sign or gust addition is incorrect.');
end
metrics=struct('AltitudesAGLCheckedM',heightsAGL,'HeldGustNEDMps',heldGust);
end

function W=fourierWeather(W)
W.GustEnabled=1;
W.GustRmsNED=[1;2;0.5];
W.GustFrequenciesHz=[0.1;0.2;0.4];
W.GustWeights=[1;0.5;0.25];
W.GustPhases=[0.1 0.3 0.7;1.1 -0.7 0.2;2.4 0.5 -1.2];
W.GustStartTime=1;
W.GustRampTime=2;
W.DiscreteGustPeakNED=zeros(3,1);
end

function metrics=checkFourier(W)
W=fourierWeather(W);
times=(20:0.01:40).'; % Two common periods; ramp is complete.
gust=zeros(numel(times),3);
replay=zeros(size(gust));
for k=1:numel(times), gust(k,:)=rocketWeatherGust(times(k),W).'; end
for k=numel(times):-1:1, replay(k,:)=rocketWeatherGust(times(k),W).'; end
actualRms=sqrt(trapz(times,gust.^2,1)/(times(end)-times(1))).';
average=trapz(times,gust,1)/(times(end)-times(1));
assert(isequal(gust,replay) && norm(actualRms-W.GustRmsNED)<1e-9 && norm(average)<1e-10, ...
    'RocketWeatherCheck:FourierRMS','Deterministic Fourier gust has hidden state, incorrect RMS normalization or a mean offset.');
completed=W;
% Preserve onset time because Fourier phases are relative to that onset;
% only shorten the envelope, otherwise this would compare different waves.
completed.GustRampTime=0.1;
atStart=rocketWeatherGust(1,W);
beforeStart=rocketWeatherGust(0.9,W);
atMiddle=rocketWeatherGust(2,W);
atEnd=rocketWeatherGust(3,W);
assert(norm(atStart)==0 && norm(beforeStart)==0 && ...
    norm(atMiddle-0.5*rocketWeatherGust(2,completed))<1e-12 && ...
    norm(atEnd-rocketWeatherGust(3,completed))<1e-12, ...
    'RocketWeatherCheck:GustRamp','One-cosine ramp has incorrect onset, midpoint or completed amplitude.');
smallTime=1e-4;
startSlope=norm(rocketWeatherGust(1+smallTime,W))/smallTime;
endSlopeDifference=norm(rocketWeatherGust(3-smallTime,W)- ...
    rocketWeatherGust(3-smallTime,completed))/smallTime;
assert(startSlope<0.002 && endSlopeDifference<0.002, ...
    'RocketWeatherCheck:GustRampSmooth','Ramp amplitude derivative is not continuous at its endpoints.');
metrics=struct('RequestedRmsNEDMps',W.GustRmsNED,'MeasuredRmsNEDMps',actualRms, ...
    'MeanGustNEDMps',average.','OnsetSlopeNormMps2',startSlope, ...
    'CompletedSlopeDifferenceMps2',endSlopeDifference,'ReplayExactlyEqual',true);
end

function metrics=checkPulse(W)
W.GustEnabled=1;
W.GustRmsNED=zeros(3,1);
W.DiscreteGustStartTime=5;
W.DiscreteGustDuration=4;
W.DiscreteGustPeakNED=[3;-1;2];
times=[4;5;5.001;6;7;8;8.999;9;10];
actual=zeros(numel(times),3);
for k=1:numel(times)
    fraction=(times(k)-5)/4;
    expected=zeros(3,1);
    if fraction>=0 && fraction<=1
        expected=W.DiscreteGustPeakNED*0.5*(1-cos(2*pi*fraction));
    end
    actual(k,:)=rocketWeatherGust(times(k),W).';
    assert(norm(actual(k,:).'-expected)<1e-12, ...
        'RocketWeatherCheck:DiscretePulse','Finite gust duration, peak, NED signs or endpoint support is incorrect.');
end
smallTime=1e-5;
assert(norm(rocketWeatherGust(5+smallTime,W))/smallTime<1e-3 && ...
    norm(rocketWeatherGust(9-smallTime,W))/smallTime<1e-3, ...
    'RocketWeatherCheck:PulseSmooth','Finite gust does not approach zero derivative at both endpoints.');
metrics=struct('TimeS',times,'GustNEDMps',actual,'PeakNEDMps',actual(5,:).');
end

function metrics=checkLoads(R,W)
W=profileWeather(W,R);
W.WindAltitudeAGL=[0;2000];
W.WindNED=[0 0 0;40 0 0];
gust=[1;-0.5;0];
R.IgnitionTime=1e6;
R.AeroSurfaceCNa(:)=0;
R.AeroFinPlanform(:)=0;
R.AeroFinLiftMultiplier(:)=0;
R.AeroSurfaceCNa(1)=2;
R.AeroSurfaceCP(1,:)=[0 0 100];
R.AeroCdPowered(:)=0;
R.AeroCdCoast(:)=0;
R.AeroRollForceScale(:)=0;
R.AeroRollDampScale(:)=0;
R.AngularDamping=zeros(3,1);
R.IMUOffset=zeros(3,1);
x=freeState(R);
x(4:6)=[20;0;100];
h=R.AltitudeMSL+1000;
[wind,rho,p,~,~,~]=rocketWeatherConditions(h,R,W,gust);
surfaceVelocity=[1;0.5;100]; % CP at 900 m AGL: mean North wind 18 + gust 1.
transverse=norm(surfaceVelocity(1:2));
speed=norm(surfaceVelocity);
expectedForce=-0.5*rho*speed^2*R.AeroSurfaceArea(1)*2* ...
    atan2(transverse,100)*[1;0.5;0]/transverse;
[force,moment,aero]=rocketPlantAerodynamics(x,60,R,W,gust);
expectedMoment=cross([0;0;100],expectedForce);
assert(norm(wind-[21;-0.5;0])<1e-12 && norm(force-expectedForce)<1e-8 && ...
    norm(moment-expectedMoment)<1e-6 && abs(aero.speed(1)-speed)<1e-12, ...
    'RocketWeatherCheck:WeatherCP','Profile density, held gust or CP-local weather wind is inconsistent in aerodynamics.');
[aN,~,debug]=rocketPlantLoads(x,60,9.80665,R,W,gust);
[specificForce,bodyField,forceDebug]=rocketPlantForces(x,60,9.80665,[1;2;3],R,W,gust);
[~,mass]=rocketPlantProperties(60,R);
[~,~,pNEU,~,pressure]=rocketPlantKinematics(x,R,W);
assert(isequal(debug(7:8),[rho;p]) && isequal(forceDebug(7:8),[rho;p]) && ...
    abs(pressure-p)<1e-10 && pNEU(3)==h && ...
    norm(aN-expectedForce/mass-[0;0;9.80665])<1e-10 && ...
    norm(specificForce-expectedForce/mass)<1e-10 && norm(bodyField-[1;2;3])<1e-12, ...
    'RocketWeatherCheck:PressureTruth','Thermodynamics, kinematic pressure or IMU truth differs between weather consumers.');
metrics=struct('DensityKgpm3',rho,'PressurePa',pressure,'HeldGustNEDMps',gust, ...
    'CPAirVelocityBodyMps',surfaceVelocity,'ComponentForceBodyN',force);
end

function metrics=checkDynamics(R,W)
W=neutralWeather(W,R);
x=freeState(R);
legacy=x;
neutral=x;
empty=x;
weather=x;
disturbed=W;
disturbed.WindNED=W.WindNED+[20 0 0];
duration=0.4;
count=round(duration/R.Ts);
assert(abs(count*R.Ts-duration)<1e-12,'RocketWeatherCheck:FixtureStep','Short test requires a 0.4 s output grid.');
for k=1:count
    time=60+(k-1)*R.Ts;
    legacy=rocketPlantStep(legacy,time,9.80665,R);
    empty=rocketPlantStep(empty,time,9.80665,R,[],zeros(3,1));
    neutral=rocketPlantStep(neutral,time,9.80665,R,W,zeros(3,1));
    weather=rocketPlantStep(weather,time,9.80665,R,disturbed,zeros(3,1));
end
assert(isequal(legacy,empty) && isequal(legacy,neutral) && ...
    all(isfinite(weather)) && norm(weather(1:6)-neutral(1:6))>1e-3, ...
    'RocketWeatherCheck:ShortDynamics','Neutral weather changes legacy dynamics, or mean wind does not affect physical flight loads.');
sampleWeather=fourierWeather(W);
sampleWeather.GustFrequenciesHz=[1;3;7];
sampleWeather.GustStartTime=0;
sampleWeather.GustRampTime=0.1;
sampleTime=60.137;
sampled=rocketWeatherGust(sampleTime,sampleWeather);
automatic=rocketPlantStep(x,sampleTime,9.80665,R,sampleWeather);
explicit=rocketPlantStep(x,sampleTime,9.80665,R,sampleWeather,sampled);
assert(isequal(automatic,explicit), ...
    'RocketWeatherCheck:MajorSampleHold','Omitted gust is not sampled once at the major step and held through RK4 substeps.');
repeat=rocketPlantStep(x,sampleTime,9.80665,R,sampleWeather);
assert(isequal(automatic,repeat), ...
    'RocketWeatherCheck:DeterministicStep','Identical explicit weather/state/time does not reproduce dynamics.');
metrics=struct('DurationS',duration,'Samples',count,'NeutralExactlyLegacy',true, ...
    'WeatherPositionChangeM',norm(weather(1:3)-neutral(1:3)), ...
    'WeatherVelocityChangeMps',norm(weather(4:6)-neutral(4:6)), ...
    'SampledGustNEDMps',sampled,'MajorSampleHoldExactlyEqual',true);
end

function metrics=checkInvalid(R,W)
W=profileWeather(W,R);
names={'negativeTemperature','zeroPressure','RHAboveOne','zeroGridStep', ...
    'unsortedWindHeights','windWrongColumns','zeroGustFrequency', ...
    'gustPhaseShape','zeroGustWeights'};
cases=cell(size(names));
cases{1}=W; cases{1}.StationTemperatureK=-1;
cases{2}=W; cases{2}.StationPressurePa=0;
cases{3}=W; cases{3}.StationRelativeHumidity=1.1;
cases{4}=W; cases{4}.ProfileStepAltitude=0;
cases{5}=W; cases{5}.WindAltitudeAGL=flipud(W.WindAltitudeAGL);
cases{6}=W; cases{6}.WindNED=W.WindNED(:,1:2);
cases{7}=W; cases{7}.GustFrequenciesHz(1)=0;
cases{8}=W; cases{8}.GustPhases=W.GustPhases(:,1:2);
cases{9}=W; cases{9}.GustWeights(:)=0;
for k=1:numel(cases)
    rejected=false;
    try
        rocketWeatherValidateConfig(cases{k});
    catch
        rejected=true;
    end
    assert(rejected,'RocketWeatherCheck:InvalidAccepted','Invalid weather case %s was accepted.',names{k});
end
metrics=struct('RejectedCases',{names});
end

function metrics=checkSource(navigationDirectory)
files=dir(fullfile(navigationDirectory,'functions','aux_Rocket','rocketWeather*.m'));
assert(numel(files)>=5,'RocketWeatherCheck:MissingHelpers','Weather helper source files are missing.');
for k=1:numel(files)
    source=fileread(fullfile(files(k).folder,files(k).name));
    assert(isempty(regexp(source,'(?m)^\s*(persistent|global)\b','once')) && ...
        isempty(regexp(source,'\b(rand|randn|rng|evalin|assignin|readmatrix|readtable)\s*\(','once')) && ...
        isempty(regexp(source,'\bcoder\.extrinsic\b','once')), ...
        'RocketWeatherCheck:HiddenState','%s contains RNG, hidden state, workspace access or extrinsics.',files(k).name);
end
metrics=struct('FilesChecked',numel(files),'HiddenStateAndRNGAbsent',true);
end
