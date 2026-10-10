function windNED = rocketWeatherWind(altitudeMSL,Rocket,Weather,gustNED)
%ROCKETWEATHERWIND Mean air velocity at AGL height plus explicit gust [m/s].
% The wind points TOWARD North/East/Down; it is subtracted from vehicle
% velocity for aerodynamic loads. Knots are relative to launch-site MSL,
% with endpoint clamping as in the original rocket mean-wind profile.
% Gust has no hidden time/state dependency and is passed explicitly to every
% CM/surface load evaluation; mean wind can vary with each surface altitude.
if nargin<3, Weather=[]; end
if nargin<4, gustNED=zeros(3,1); end
if isempty(Weather)
    grid=Rocket.WindAltitude;
    values=Rocket.WindNED;
else
    grid=Weather.WindAltitudeAGL;
    values=Weather.WindNED;
end
heightAGL=altitudeMSL-Rocket.AltitudeMSL;
windNED=zeros(3,1);
for k=1:3
    windNED(k)=rocketPlantInterpolate(grid,values(:,k),heightAGL);
end
windNED=windNED+gustNED(:);
end
