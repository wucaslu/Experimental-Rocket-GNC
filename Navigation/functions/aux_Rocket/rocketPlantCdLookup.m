function [cd,query] = rocketPlantCdLookup(speedAirMps,controlLevel,lookup)
%ROCKETPLANTCDLOOKUP Absolute Cd at CM airspeed and control level.
% [cd,query] = rocketPlantCdLookup(speedAirMps,controlLevel,CdLookup)
% uses bilinear interpolation: rows=airspeed [m/s], columns=control 0:0.1:1.
% Finite queries outside the axes are clipped, with no extrapolation.
% query=[usedSpeed;usedControl]. Between control nodes the response is smooth;
% the 0.1 spacing describes the table, not quantization of the input command.
% No persistent variables or hidden state. Coefficients are dimensionless.
rocketPlantValidateCdLookup(lookup);
if ~isnumeric(speedAirMps) || ~isscalar(speedAirMps) || ~isreal(speedAirMps) || ...
        ~isfinite(speedAirMps) || speedAirMps<0 || ...
        ~isnumeric(controlLevel) || ~isscalar(controlLevel) || ~isreal(controlLevel) || ...
        ~isfinite(controlLevel)
    error('RocketPlant:InvalidCdLookupInput', ...
        'Airspeed must be finite, real, scalar and nonnegative; control must be a finite real numeric scalar.');
end
v=double(lookup.VelocityBreakpoints);
u=double(lookup.ControlBreakpoints);
table=double(lookup.Table);
speed=min(v(end),max(v(1),double(speedAirMps)));
control=min(u(end),max(u(1),double(controlLevel)));
[i,a]=interval(v,speed);
[j,b]=interval(u,control);
low=table(i,j)+b*(table(i,j+1)-table(i,j));
high=table(i+1,j)+b*(table(i+1,j+1)-table(i+1,j));
cd=low+a*(high-low);
query=[speed;control];
end

function [index,fraction]=interval(axis,value)
index=1;
for k=1:numel(axis)-1
    index=k;
    if value<=axis(k+1), break; end
end
fraction=(value-axis(index))/(axis(index+1)-axis(index));
end
