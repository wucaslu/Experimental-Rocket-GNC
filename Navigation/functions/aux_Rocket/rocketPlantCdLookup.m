function [cd,query] = rocketPlantCdLookup(speedAirMps,controlLevel,lookup,soundSpeed)
%ROCKETPLANTCDLOOKUP Absolute grid Cd or effective added airbrake Cd.
% [cd,query] = rocketPlantCdLookup(speedAirMps,controlLevel,CdLookup)
% uses bilinear interpolation: rows=airspeed [m/s], columns=control 0:0.1:1.
% Finite queries outside the axes are clipped, with no extrapolation.
% query=[usedSpeed;usedControl]. Between control nodes the response is smooth;
% the 0.1 spacing describes the table, not quantization of the input command.
% A Mode=1 scattered source returns added Cd using RocketPy's all-point
% Shepard interpolation in deployment/Mach coordinates. soundSpeed [m/s]
% converts airspeed to Mach (default 340); Mach is not clipped. Natural
% extrapolation uses the same inverse-distance law. Deployment is bounded
% to [0,1], and zero deployment produces zero effective airbrake Cd even
% when the raw interpolated coefficient is nonzero. query remains [m/s;-].
% No persistent variables or hidden state. Coefficients are dimensionless.
if nargin<4, soundSpeed=340; end
rocketPlantValidateCdLookup(lookup);
if ~isnumeric(speedAirMps) || ~isscalar(speedAirMps) || ~isreal(speedAirMps) || ...
        ~isfinite(speedAirMps) || speedAirMps<0 || ...
        ~isnumeric(controlLevel) || ~isscalar(controlLevel) || ~isreal(controlLevel) || ...
        ~isfinite(controlLevel)
    error('RocketPlant:InvalidCdLookupInput', ...
        'Airspeed must be finite, real, scalar and nonnegative; control must be a finite real numeric scalar.');
end
if isfield(lookup,'Mode')
    if ~isnumeric(soundSpeed) || ~isreal(soundSpeed) || ~isscalar(soundSpeed) || ...
            ~isfinite(soundSpeed) || soundSpeed<=0
        error('RocketPlant:InvalidCdLookupInput','Sound speed must be a finite positive real numeric scalar.');
    end
    speed=double(speedAirMps);
    control=min(1,max(0,double(controlLevel)));
    query=[speed;control];
    cd=0;
    if control==0, return; end
    samples=double(lookup.Samples);
    mach=speed/double(soundSpeed);
    if ~isfinite(mach)
        error('RocketPlant:InvalidCdLookupInput','Airspeed divided by sound speed must be finite.');
    end
    distances=hypot(samples(:,1)-control,samples(:,2)-mach);
    minimumDistance=min(distances);
    % Scaling all weights by the nearest distance prevents overflow
    % arbitrarily close to a sample without changing the Shepard ratio.
    for k=1:size(samples,1)
        if distances(k)==0
            cd=samples(k,3);
            return
        end
    end
    weights=(minimumDistance./distances).^3;
    cd=sum(samples(:,3).*weights)/sum(weights);
    return
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
