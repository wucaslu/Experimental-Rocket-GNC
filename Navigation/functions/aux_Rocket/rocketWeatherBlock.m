function rocketWeatherBlock(block)
%ROCKETWEATHERBLOCK Stateless Level-2 adapter for Environment weather truth.
% Dialog: Rocket, Weather, TsRocket. Inputs: position_NEU3 [m] (absolute Up
% is MSL altitude), model time [s]. Outputs: wind_NED3 [m/s], atmosphere5
% [rho;pressure;soundSpeed;temperature;RH], gust_NED3 [m/s]. Gust is exported
% explicitly for the plant dynamics; this block has no internal flight state.
block.NumDialogPrms=3;
block.NumInputPorts=2;
block.NumOutputPorts=3;
block.SetPreCompPortInfoToDefaults;
inputWidths=[3 1];
outputWidths=[3 5 3];
for k=1:2
    block.InputPort(k).Dimensions=inputWidths(k);
    block.InputPort(k).DatatypeID=0;
    block.InputPort(k).Complexity='Real';
    block.InputPort(k).DirectFeedthrough=true;
end
for k=1:3
    block.OutputPort(k).Dimensions=outputWidths(k);
    block.OutputPort(k).DatatypeID=0;
    block.OutputPort(k).Complexity='Real';
end
block.SampleTimes=[block.DialogPrm(3).Data 0];
block.SimStateCompliance='HasNoSimState';
block.RegBlockMethod('CheckParameters',@checkParameters);
block.RegBlockMethod('Outputs',@outputs);
end

function checkParameters(block)
rocketWeatherValidateConfig(block.DialogPrm(2).Data);
sampleTime=block.DialogPrm(3).Data;
assert(isnumeric(sampleTime) && isreal(sampleTime) && isscalar(sampleTime) && ...
    isfinite(sampleTime) && sampleTime>0, ...
    'RocketWeather:InvalidSampleTime','TsRocket must be a finite positive sample time [s].');
end

function outputs(block)
Rocket=block.DialogPrm(1).Data;
Weather=block.DialogPrm(2).Data;
positionNEU=block.InputPort(1).Data;
gustNED=rocketWeatherGust(block.InputPort(2).Data,Weather);
[windNED,rho,pressure,soundSpeed,temperature,RH]= ...
    rocketWeatherConditions(positionNEU(3),Rocket,Weather,gustNED);
block.OutputPort(1).Data=windNED;
block.OutputPort(2).Data=[rho;pressure;soundSpeed;temperature;RH];
block.OutputPort(3).Data=gustNED;
end
