function rocketWeatherForecastInitialize(originalInitFcn,dateTimeUTC)
%ROCKETWEATHERFORECASTINITIALIZE Apply a forecast date before model initialization.
% Inputs: originalInitFcn, the model's existing initialization callback text;
% dateTimeUTC, resolved ISO UTC text for the already validated cached snapshot.
% Output: no returned value; initializes the model's base-workspace parameters.
% SimulationInput variable overrides become visible after InitFcn in this
% model, so the temporary callback must supply the date before that callback.
% Explicit temporary inputs are restored even when initialization fails.
% No persistent variables or network calls are used by this callback helper;
% rocketPlantMain loads the selected numeric profile through the MATLAB loader.
assert(ischar(originalInitFcn) && isrow(originalInitFcn) && ...
    (ischar(dateTimeUTC) || (isstring(dateTimeUTC) && isscalar(dateTimeUTC))), ...
    'RocketWeatherForecast:InvalidInitialization','Use callback text and a resolved UTC date.');
names={'RocketWeatherForecastDateTimeUTC','RocketWeatherForecastCacheMode'};
previous=cell(size(names));
present=false(size(names));
for k=1:numel(names)
    present(k)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{k}));
    if present(k), previous{k}=evalin('base',names{k}); end
end
restore=onCleanup(@() restoreInputs(names,present,previous));
assignin('base',names{1},char(dateTimeUTC));
assignin('base',names{2},'cache');
evalin('base',originalInitFcn);
end

function restoreInputs(names,present,previous)
for k=1:numel(names)
    if present(k), assignin('base',names{k},previous{k});
    else, evalin('base',['clear ' names{k}]); end
end
end
