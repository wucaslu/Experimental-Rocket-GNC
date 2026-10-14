function Weather = rocketMonteCarloWeather(nominalWeather,nominalRocket,Rocket,sample)
%ROCKETMONTECARLOWEATHER Perturb one frozen atmospheric profile for a run.
% Weather = rocketMonteCarloWeather(nominalWeather,nominalRocket,Rocket,sample)
% prepares numeric Environment data before simulation. It reads no forecast,
% file, global RNG or persistent state, and does not change onboard guidance.
%
% Inputs:
%   nominalWeather: validated frozen Weather struct. Profile columns contain
%       geometric MSL height [m], pressure [Pa], temperature [K] and RH [-].
%       WindNED is air velocity TOWARD North/East/Down [m/s] at AGL knots.
%   nominalRocket, Rocket: nominal and sampled launch configurations. Their
%       AltitudeMSL [m] anchors the local site; IgnitionTime [s] shifts gust
%       onset/pulse times while preserving their time relative to ignition.
%   sample: scalar struct with TemperatureOffsetK [K], PressureScale (>0),
%       LapseRateOffsetKpm [K/m of geopotential height], RelativeHumidityOffset
%       [fraction], WindOffsetNED [3x1 m/s], WindShearNED [3x1 (m/s)/km AGL],
%       GustRmsNED [3x1 m/s of ADDITIONAL nonnegative stationary RMS], and
%       GustPhases [Kx3 radians, one row per configured Fourier frequency].
% Output:
%   Weather: one numeric profile with coherent P/T/RH and derived moist
%       density/sound speed, plus mean wind and explicit-time synthetic gusts.
%       Existing atmospheric altitude bounds and wind AGL knots are retained.
%
% Thermodynamics preserve the imported forecast's shape, rather than
% replacing it with ISA. With H=R*z/(R+z), R=6356766 m, define
%   T'=T0 + dT + dL*(min(H,11000)-min(Hstation,11000)),
%   RH'=clip(RH0+dRH*exp(-max(z-zstation,0)/HumidityScaleHeight),0,1).
% For r=p'/p0, integrate from the sampled station using RK4:
%   d(log(r))/dH=-g0*(rho(p0*r,T',RH')/(p0*r)-rho(p0,T0,RH0)/p0),
%   log(r_station)=log(PressureScale), g0=9.80665 m/s^2.
% This preserves baseline pressure-gradient departures from ideal moist
% hydrostatics and changes only their thermodynamic contribution. It is not
% an assertion that the imported forecast itself is exactly hydrostatic.
% Pressure is not perturbed independently of density or sound speed. The
% same Tetens/ideal-mixture relations as rocketWeatherConditions are used.
% The unchanged thermal case at an unchanged site copies the imported
% P/T/RH/rho/sound tables and their interpolation mode exactly. Other cases
% set UseTabulatedThermodynamics=0 so runtime interpolated P/T/RH determine
% density and sound speed together. Archived rho/sound columns, if present,
% are recalculated on the new grid for consistent dormant numeric fields.
%
% Additional gust RMS combines in quadrature with the baseline RMS. This
% describes an ensemble of the existing finite Fourier gust approximation,
% not independently superposed turbulence records or a Dryden spectrum.
% Gust phases are supplied explicitly; finite-flight RMS can differ from
% the configured infinite-time RMS. Temperature/pressure draws outside the
% model's physical bounds raise an error; RH is deliberately clipped.

rocketWeatherValidateConfig(nominalWeather,true);
validateInputs(nominalWeather,nominalRocket,Rocket,sample);
Weather=nominalWeather;

% Wind axes are already local AGL; retain the imported mean profile and add
% a site wind offset plus a linear local-height shear. Down-positive signs
% are preserved, so an upward air velocity is a negative Down component.
Weather.WindNED=nominalWeather.WindNED+ ...
    ones(numel(Weather.WindAltitudeAGL),1)*sample.WindOffsetNED.'+ ...
    (Weather.WindAltitudeAGL/1000)*sample.WindShearNED.';
Weather.GustPhases=sample.GustPhases;
if any(sample.GustRmsNED>0)
    Weather.GustRmsNED=hypot(nominalWeather.GustRmsNED,sample.GustRmsNED);
    Weather.GustEnabled=1;
end
ignitionShift=Rocket.IgnitionTime-nominalRocket.IgnitionTime;
Weather.GustStartTime=nominalWeather.GustStartTime+ignitionShift;
Weather.DiscreteGustStartTime=nominalWeather.DiscreteGustStartTime+ignitionShift;

unchangedThermal=sample.TemperatureOffsetK==0 && sample.PressureScale==1 && ...
    sample.LapseRateOffsetKpm==0 && sample.RelativeHumidityOffset==0 && ...
    Rocket.AltitudeMSL==nominalRocket.AltitudeMSL;
if unchangedThermal
    rocketWeatherValidateConfig(Weather,true);
    return
end

% Insert the sampled station as an exact pressure-ratio anchor. No new
% profile range or extrapolation is invented when changing site altitude.
altitude=unique([nominalWeather.ProfileAltitudeMSL;Rocket.AltitudeMSL]);
radius=6356766;
height=radius*altitude./(radius+altitude);
station=find(altitude==Rocket.AltitudeMSL,1);
stationHeight=height(station);
baseline=[nominalWeather.ProfilePressurePa, ...
    nominalWeather.ProfileTemperatureK,nominalWeather.ProfileRelativeHumidity];
nominalValues=interp1(nominalWeather.ProfileAltitudeMSL,baseline,altitude,'linear');
temperature=nominalValues(:,2)+sample.TemperatureOffsetK+ ...
    sample.LapseRateOffsetKpm*(min(height,11000)-min(stationHeight,11000));
relativeHumidity=min(1,max(0,nominalValues(:,3)+sample.RelativeHumidityOffset* ...
    exp(-max(altitude-Rocket.AltitudeMSL,0)/Weather.HumidityScaleHeight)));
assert(all(isfinite(temperature) & temperature>=180 & temperature<=350), ...
    'RocketMonteCarlo:InvalidWeatherTemperature', ...
    'Sampled atmospheric temperatures must remain between 180 and 350 K.');

logRatio=zeros(size(height));
logRatio(station)=log(sample.PressureScale);
for k=station:numel(height)-1
    logRatio(k+1)=ratioStep(height(k),logRatio(k),height(k+1), ...
        nominalWeather.ProfileAltitudeMSL,baseline,Rocket.AltitudeMSL, ...
        stationHeight,Weather.HumidityScaleHeight,sample);
end
for k=station:-1:2
    logRatio(k-1)=ratioStep(height(k),logRatio(k),height(k-1), ...
        nominalWeather.ProfileAltitudeMSL,baseline,Rocket.AltitudeMSL, ...
        stationHeight,Weather.HumidityScaleHeight,sample);
end
pressure=nominalValues(:,1).*exp(logRatio);
[density,soundSpeed]=moistProperties(pressure,temperature,relativeHumidity);

Weather.UseProfile=1;
Weather.UseTabulatedThermodynamics=0;
Weather.StationAltitudeMSL=Rocket.AltitudeMSL;
Weather.StationTemperatureK=temperature(station);
Weather.StationPressurePa=pressure(station);
Weather.StationRelativeHumidity=relativeHumidity(station);
Weather.LapseRateOffset=nominalWeather.LapseRateOffset+sample.LapseRateOffsetKpm;
Weather.ProfileAltitudeMSL=altitude;
Weather.ProfilePressurePa=pressure;
Weather.ProfileTemperatureK=temperature;
Weather.ProfileRelativeHumidity=relativeHumidity;
if isfield(Weather,'ProfileDensityKgpm3')
    Weather.ProfileDensityKgpm3=density;
end
if isfield(Weather,'ProfileSoundSpeedMps')
    Weather.ProfileSoundSpeedMps=soundSpeed;
end
rocketWeatherValidateConfig(Weather,true);
end

function next=ratioStep(startHeight,startRatio,endHeight,altitude,baseline, ...
    stationAltitude,stationHeight,humidityScaleHeight,sample)
% Integrate in either direction with <=25 m geopotential substeps. Fixed
% small steps keep pressure/RH coupling resolved even with coarse soundings.
substeps=max(1,ceil(abs(endHeight-startHeight)/25));
step=(endHeight-startHeight)/substeps;
next=startRatio;
for k=1:substeps
    h=startHeight+(k-1)*step;
    k1=ratioGradient(h,next,altitude,baseline,stationAltitude,stationHeight,humidityScaleHeight,sample);
    k2=ratioGradient(h+step/2,next+step*k1/2,altitude,baseline,stationAltitude,stationHeight,humidityScaleHeight,sample);
    k3=ratioGradient(h+step/2,next+step*k2/2,altitude,baseline,stationAltitude,stationHeight,humidityScaleHeight,sample);
    k4=ratioGradient(h+step,next+step*k3,altitude,baseline,stationAltitude,stationHeight,humidityScaleHeight,sample);
    next=next+step*(k1+2*k2+2*k3+k4)/6;
    assert(isfinite(next),'RocketMonteCarlo:InvalidWeatherPressure', ...
        'The sampled pressure-ratio integration became nonfinite.');
end
end

function gradient=ratioGradient(height,logRatio,altitude,baseline, ...
    stationAltitude,stationHeight,humidityScaleHeight,sample)
radius=6356766;
z=radius*height/(radius-height);
% Inverse geopotential mapping can round by a few ulps at a profile bound.
z=min(altitude(end),max(altitude(1),z));
nominal=interp1(altitude,baseline,z,'linear');
temperature=nominal(2)+sample.TemperatureOffsetK+ ...
    sample.LapseRateOffsetKpm*(min(height,11000)-min(stationHeight,11000));
humidity=min(1,max(0,nominal(3)+sample.RelativeHumidityOffset* ...
    exp(-max(z-stationAltitude,0)/humidityScaleHeight)));
pressure=nominal(1)*exp(logRatio);
nominalDensity=moistProperties(nominal(1),nominal(2),nominal(3));
density=moistProperties(pressure,temperature,humidity);
gradient=-9.80665*(density/pressure-nominalDensity/nominal(1));
end

function [density,soundSpeed]=moistProperties(pressure,temperature,humidity)
% The same ideal-mixture law as rocketWeatherConditions, evaluated at nodes
% or RK4 stages. Partial pressures, density and sound speed are coherent.
tc=temperature-273.15;
vapor=humidity.*611.*10.^(7.5.*tc./(237.3+tc));
assert(all(isfinite(pressure(:)) & pressure(:)>0) && ...
    all(isfinite(temperature(:)) & temperature(:)>=180 & temperature(:)<=350) && ...
    all(isfinite(vapor(:)) & vapor(:)<pressure(:)), ...
    'RocketMonteCarlo:InvalidWeatherPressure', ...
    'Sampled atmosphere needs finite physical temperature and positive dry-air partial pressure.');
dryDensity=(pressure-vapor)./(287.05287.*temperature);
vaporDensity=vapor./(461.5.*temperature);
density=dryDensity+vaporDensity;
specificHumidity=vaporDensity./density;
gasConstant=(1-specificHumidity).*287.05287+specificHumidity.*461.5;
cp=(1-specificHumidity).*(3.5*287.05287)+specificHumidity.*1850;
soundSpeed=sqrt((cp./(cp-gasConstant)).*gasConstant.*temperature);
assert(all(isfinite(density(:)) & density(:)>0) && ...
    all(isfinite(soundSpeed(:)) & soundSpeed(:)>0), ...
    'RocketMonteCarlo:InvalidWeatherPressure','Sampled density and sound speed must be finite and positive.');
end

function validateInputs(Weather,nominalRocket,Rocket,sample)
assert(isstruct(nominalRocket) && isscalar(nominalRocket) && ...
    isstruct(Rocket) && isscalar(Rocket) && ...
    all(isfield(nominalRocket,{'AltitudeMSL','IgnitionTime'})) && ...
    all(isfield(Rocket,{'AltitudeMSL','IgnitionTime'})) && ...
    finiteScalar(nominalRocket.AltitudeMSL) && finiteScalar(Rocket.AltitudeMSL) && ...
    finiteScalar(nominalRocket.IgnitionTime) && finiteScalar(Rocket.IgnitionTime), ...
    'RocketMonteCarlo:InvalidWeatherRocket','Nominal and sampled rockets need finite launch altitude and ignition time.');
assert(all(isfield(Weather,{'ProfileAltitudeMSL','ProfilePressurePa', ...
    'ProfileTemperatureK','ProfileRelativeHumidity'})), ...
    'RocketMonteCarlo:MissingWeatherProfile','The frozen nominal atmosphere must contain P/T/RH profile columns.');
assert(Rocket.AltitudeMSL>=Weather.ProfileAltitudeMSL(1) && ...
    Rocket.AltitudeMSL<=Weather.ProfileAltitudeMSL(end) && ...
    Rocket.AltitudeMSL>=Weather.ProfileMinAltitudeMSL && ...
    Rocket.AltitudeMSL<=Weather.ProfileMaxAltitudeMSL, ...
    'RocketMonteCarlo:WeatherSiteOutsideProfile','Sampled launch altitude must remain inside the frozen profile bounds.');
fields={'TemperatureOffsetK','PressureScale','LapseRateOffsetKpm', ...
    'RelativeHumidityOffset','WindOffsetNED','WindShearNED','GustRmsNED','GustPhases'};
assert(isstruct(sample) && isscalar(sample) && all(isfield(sample,fields)), ...
    'RocketMonteCarlo:InvalidWeatherSample','A complete scalar weather sample struct is required.');
for name=fields(1:4)
    assert(finiteScalar(sample.(name{1})), ...
        'RocketMonteCarlo:InvalidWeatherSample','Sample.%s must be a finite real scalar.',name{1});
end
assert(sample.PressureScale>0,'RocketMonteCarlo:InvalidWeatherSample', ...
    'PressureScale must be strictly positive.');
for name=fields(5:7)
    value=sample.(name{1});
    assert(isnumeric(value) && isreal(value) && isequal(size(value),[3 1]) && ...
        all(isfinite(value)),'RocketMonteCarlo:InvalidWeatherSample', ...
        'Sample.%s must be a finite real 3x1 vector.',name{1});
end
assert(all(sample.GustRmsNED>=0) && isnumeric(sample.GustPhases) && ...
    isreal(sample.GustPhases) && isequal(size(sample.GustPhases),size(Weather.GustPhases)) && ...
    all(isfinite(sample.GustPhases(:))), ...
    'RocketMonteCarlo:InvalidWeatherSample','Additional gust RMS must be nonnegative and phases a finite matching Kx3 array.');
end

function yes=finiteScalar(value)
yes=isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end
