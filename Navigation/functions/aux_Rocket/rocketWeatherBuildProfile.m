function Weather = rocketWeatherBuildProfile(Weather)
%ROCKETWEATHERBUILDPROFILE Build a station-anchored moist hydrostatic profile.
% Geometric MSL grid is mapped to geopotential H, matching the legacy ISA.
% T=T_ISA+station offset+lapse offset*troposphere height above the station.
% RH is prescribed over liquid water and decays exponentially above station.
% Integrate dp/dH=-g0*rho by RK4 in both directions from station pressure;
% rho=(p-e)/(Rd*T)+e/(Rv*T). This includes moisture in hydrostatics itself.
% Profile construction occurs at initialization, never during a plant step.
% This ideal-gas profile omits condensation, ice-phase RH and real weather
% gradients. Its output columns may later be replaced by measured soundings.
% https://www.ncl.ucar.edu/Document/Functions/Built-in/hydro.shtml
% https://www.weather.gov/media/epz/wxcalc/vaporPressure.pdf
rocketWeatherValidateConfig(Weather,false);
radius=6356766;
lower=Weather.ProfileMinAltitudeMSL;
upper=Weather.ProfileMaxAltitudeMSL;
grid=(lower:Weather.ProfileStepAltitude:upper).';
layerH=[0;11000;20000;32000];
layerZ=radius*layerH./(radius-layerH);
layerZ=layerZ(layerZ>lower & layerZ<upper);
grid=unique([grid;upper;Weather.StationAltitudeMSL;layerZ]);
H=radius*grid./(radius+grid);
station=find(grid==Weather.StationAltitudeMSL,1);
n=numel(grid);
pressure=zeros(n,1);
temperature=zeros(n,1);
relativeHumidity=zeros(n,1);
[~,~,~,stationISA]=rocketPlantAtmosphere(Weather.StationAltitudeMSL);
temperatureOffset=Weather.StationTemperatureK-stationISA;
stationH=radius*Weather.StationAltitudeMSL/(radius+Weather.StationAltitudeMSL);
for k=1:n
    [temperature(k),relativeHumidity(k)]=thermalProfile(H(k), ...
        Weather,temperatureOffset,stationH);
end
pressure(station)=Weather.StationPressurePa;
for k=station:n-1
    pressure(k+1)=pressureStep(H(k),pressure(k),H(k+1)-H(k), ...
        Weather,temperatureOffset,stationH);
end
for k=station:-1:2
    pressure(k-1)=pressureStep(H(k),pressure(k),H(k-1)-H(k), ...
        Weather,temperatureOffset,stationH);
end
Weather.ProfileAltitudeMSL=grid;
Weather.ProfilePressurePa=pressure;
Weather.ProfileTemperatureK=temperature;
Weather.ProfileRelativeHumidity=relativeHumidity;
rocketWeatherValidateConfig(Weather,true);
end

function pNext=pressureStep(H,p,step,Weather,offset,stationH)
k1=pressureGradient(H,p,Weather,offset,stationH);
k2=pressureGradient(H+step/2,p+step*k1/2,Weather,offset,stationH);
k3=pressureGradient(H+step/2,p+step*k2/2,Weather,offset,stationH);
k4=pressureGradient(H+step,p+step*k3,Weather,offset,stationH);
pNext=p+step*(k1+2*k2+2*k3+k4)/6;
end

function gradient=pressureGradient(H,p,Weather,offset,stationH)
[T,RH]=thermalProfile(H,Weather,offset,stationH);
tc=T-273.15;
e=RH*611*10^(7.5*tc/(237.3+tc));
assert(isfinite(p) && p>e && p>0, ...
    'RocketWeather:InvalidVaporPressure', ...
    'Station/profile conditions produce nonpositive dry-air partial pressure.');
rho=(p-e)/(287.05287*T)+e/(461.5*T);
gradient=-9.80665*rho;
end

function [T,RH]=thermalProfile(H,Weather,offset,stationH)
radius=6356766;
altitude=radius*H/(radius-H);
[~,~,~,isaT]=rocketPlantAtmosphere(altitude);
T=isaT+offset+Weather.LapseRateOffset*(min(H,11000)-min(stationH,11000));
RH=Weather.StationRelativeHumidity* ...
    exp(-max(altitude-Weather.StationAltitudeMSL,0)/Weather.HumidityScaleHeight);
assert(isfinite(T) && T>=180 && T<=350, ...
    'RocketWeather:InvalidTemperature','Temperature profile must stay between 180 and 350 K.');
end
