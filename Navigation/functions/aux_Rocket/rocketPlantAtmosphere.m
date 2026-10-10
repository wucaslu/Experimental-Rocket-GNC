function [rho,pressure,soundSpeed,temperature] = rocketPlantAtmosphere(altitudeMSL)
%ROCKETPLANTATMOSPHERE Dry 1976 standard-atmosphere layers in SI units.
% Geometric altitude is converted to geopotential altitude. This atmosphere
% intentionally omits weather, humidity and wind (wind is separate input).
radius = 6356766;
altitudeMSL=max(-5000,altitudeMSL);
h=radius*altitudeMSL/(radius+altitudeMSL);
layerH=[0;11000;20000;32000;47000;51000;71000;84852];
lapse=[-0.0065;0;0.001;0.0028;0;-0.0028;-0.002;0];
gStd=9.80665;
gasConstant=287.05287;
temperature=288.15;
pressure=101325;
baseH=0;
for i=1:8
    if i<8
        targetH=min(h,layerH(i+1));
    else
        targetH=h;
    end
    deltaH=targetH-baseH;
    newTemperature=temperature+lapse(i)*deltaH;
    if abs(lapse(i))<1e-12
        pressure=pressure*exp(-gStd*deltaH/(gasConstant*temperature));
    else
        pressure=pressure*(newTemperature/temperature)^(-gStd/(gasConstant*lapse(i)));
    end
    temperature=newTemperature;
    if h<=targetH
        break
    end
    baseH=targetH;
end
rho=pressure/(gasConstant*temperature);
soundSpeed=sqrt(1.4*gasConstant*temperature);
end
