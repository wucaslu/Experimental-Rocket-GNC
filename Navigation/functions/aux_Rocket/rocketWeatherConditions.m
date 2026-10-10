function [windNED,rho,pressure,soundSpeed,temperature,RH] = ...
    rocketWeatherConditions(altitudeMSL,Rocket,Weather,gustNED)
%ROCKETWEATHERCONDITIONS Stateless atmosphere and wind at geometric MSL [m].
% Outputs: wind NED [m/s], rho [kg/m^3], P [Pa], sound speed [m/s], T [K],
% RH fraction. Empty Weather or UseProfile=0 calls dry ISA without changes.
% With a profile, interpolate P/T/RH and recompute density and sound speed
% from one moist ideal-gas mixture, rather than interpolating them separately.
% Liquid-water Tetens saturation is approximate; no phase change is modeled.
% Constant species heat capacities approximate the sound-speed humidity effect.
% https://www.weather.gov/media/epz/wxcalc/vaporPressure.pdf
% https://www.grc.nasa.gov/www/k-12/BGP/snddrv.html
% https://ntrs.nasa.gov/api/citations/19830003823/downloads/19830003823.pdf
if nargin<3, Weather=[]; end
if nargin<4, gustNED=zeros(3,1); end
windNED=rocketWeatherWind(altitudeMSL,Rocket,Weather,gustNED);
if isempty(Weather) || Weather.UseProfile==0
    [rho,pressure,soundSpeed,temperature]=rocketPlantAtmosphere(altitudeMSL);
    RH=0;
    return
end
assert(isfinite(altitudeMSL) && altitudeMSL>=Weather.ProfileAltitudeMSL(1) && ...
    altitudeMSL<=Weather.ProfileAltitudeMSL(end), ...
    'RocketWeather:AltitudeOutsideProfile', ...
    'Geometric MSL altitude must lie inside the configured weather profile.');
% One binary search serves all three columns of the dense profile. Avoid
% scanning the table separately for every CM/fin/tail/RK4 load evaluation.
lower=1;
upper=numel(Weather.ProfileAltitudeMSL);
while upper-lower>1
    middle=floor((lower+upper)/2);
    if altitudeMSL<Weather.ProfileAltitudeMSL(middle)
        upper=middle;
    else
        lower=middle;
    end
end
fraction=(altitudeMSL-Weather.ProfileAltitudeMSL(lower))/ ...
    (Weather.ProfileAltitudeMSL(upper)-Weather.ProfileAltitudeMSL(lower));
pressure=Weather.ProfilePressurePa(lower)+fraction* ...
    (Weather.ProfilePressurePa(upper)-Weather.ProfilePressurePa(lower));
temperature=Weather.ProfileTemperatureK(lower)+fraction* ...
    (Weather.ProfileTemperatureK(upper)-Weather.ProfileTemperatureK(lower));
RH=Weather.ProfileRelativeHumidity(lower)+fraction* ...
    (Weather.ProfileRelativeHumidity(upper)-Weather.ProfileRelativeHumidity(lower));
tc=temperature-273.15;
e=RH*611*10^(7.5*tc/(237.3+tc));
assert(e<pressure && pressure>0 && temperature>0, ...
    'RocketWeather:InvalidAtmosphere','Moist mixture needs positive dry-air pressure and temperature.');
dryDensity=(pressure-e)/(287.05287*temperature);
vaporDensity=e/(461.5*temperature);
rho=dryDensity+vaporDensity;
q=vaporDensity/rho;
gasConstant=(1-q)*287.05287+q*461.5;
cp=(1-q)*(3.5*287.05287)+q*1850;
gamma=cp/(cp-gasConstant);
soundSpeed=sqrt(gamma*gasConstant*temperature);
end
