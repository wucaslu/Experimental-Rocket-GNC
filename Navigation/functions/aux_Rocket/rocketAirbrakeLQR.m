function [command,diagnostics,gainInfo] = rocketAirbrakeLQR( ...
    hAGL,vUp,previousLevel,targetAGL,CdLookup,Airbrake,LQR)
%ROCKETAIRBRAKELQR Navigation-driven finite-horizon coast airbrake guidance.
% Inputs: hAGL [m] is estimated height ABOVE LAUNCH; vUp [m/s] is estimated
% ground upward velocity. previousLevel [-] is the previous continuous
% deployment proxy, in [0,1], propagated from commands and nominal actuator
% dynamics; for a direct actuator it equals previous commanded position.
% This is onboard memory, not a truth actuator-position input. targetAGL [m] is the
% editable positive target (normally 3000 m above launch). CdLookup is the
% independent onboard absolute-Cd grid (airspeed or Mode=2 Mach) or Mode=1
% incremental airbrake calibration. Absolute grids include the level-zero Cd.
% Airbrake supplies surveyed launch MSL elevation [m], nominal post-burn
% DryMass [kg], ReferenceArea [m^2], and, for incremental data, the published
% NominalMach/NominalCdCoast curve. LQR is from airbrakeLQRSettings().
% Outputs: command [-] is a continuous position in [0,1], limited to
% MaxDeploymentRate*ControllerPeriod from previousLevel. diagnostics is the
% fixed 24-by-1 vector described below; gainInfo=[K_h;K_v;K_level;affine]
% is the first finite-horizon rate-feedback gain and affine term [1/s].
% Explicit state [height; upward velocity; deployment] is rolled forward
% with deployment RATE as the input. The nominal held-position coast is
% linearized, followed by a backward affine Riccati recursion with terminal
% height priority. At each replan the initial perturbation is zero; the
% affine term carries the target mismatch into the present command.
% Full opening/retraction forecasts include the same deployment slew.
% The static atmosphere and nominal rocket constants are ONBOARD models;
% no truth state, wind, plant weather, motor status or GNSS velocity is read.
% Enable/burnout/speed/descent latches belong in the upstream enable block.
% This helper has no persistent variables and performs no enable decisions.
% Nonascending estimates request rate-limited retraction. Invalid feedback
% holds the previous valid command; an invalid previous command returns zero.
% Saturation makes the applied controller locally LQR-based; it is not a
% constrained-optimal nonlinear controller or an apogee hover regulator.
%
% diagnostics = [valid; status; minimum reachable peak; maximum reachable
% peak; held-position peak; target; held-position peak minus target;
% raw rate; applied rate; command; rate/bound saturation; horizon steps;
% horizon seconds; K_h; K_v; K_level; affine rate; instantaneous-retracted
% peak; allowed minimum rate; allowed maximum rate; hAGL; vUp;
% previousLevel; target reachable]. Heights are AGL metres; rates are 1/s.
% status: -2 numerical/prediction failure, -1 invalid feedback, 0 already
% nonascending, 1 LQR, 2 target above reachable peak, 3 target below it.
%#codegen
validateSettings(Airbrake,LQR,CdLookup);
diagnostics=zeros(24,1);
gainInfo=zeros(4,1);
command=0;
levelValid=isnumeric(previousLevel) && isscalar(previousLevel) && ...
    isreal(previousLevel) && isfinite(previousLevel) && ...
    previousLevel>=0 && previousLevel<=1;
if levelValid, command=previousLevel; end
diagnostics(2)=-1;
diagnostics(10)=command;
feedbackValid=isnumeric(hAGL) && isscalar(hAGL) && ...
    isnumeric(vUp) && isscalar(vUp) && ...
    isnumeric(targetAGL) && isscalar(targetAGL) && ...
    isreal(hAGL) && isreal(vUp) && isreal(targetAGL) && ...
    isfinite(hAGL) && isfinite(vUp) && isfinite(targetAGL) && ...
    hAGL>=0 && targetAGL>0 && levelValid;
if ~feedbackValid, return; end
diagnostics(6)=targetAGL;
diagnostics(21:23)=[hAGL;vUp;previousLevel];
dtCommand=LQR.ControllerPeriod;
maxRate=LQR.MaxDeploymentRate;
minimumRate=max(-maxRate,-previousLevel/dtCommand);
maximumRate=min(maxRate,(1-previousLevel)/dtCommand);
diagnostics(19:20)=[minimumRate;maximumRate];
if vUp<=0
    command=max(0,previousLevel-maxRate*dtCommand);
    diagnostics(1)=1;
    diagnostics(2)=0;
    diagnostics(3:5)=hAGL;
    diagnostics(7)=hAGL-targetAGL;
    diagnostics(8:10)=[-maxRate;(command-previousLevel)/dtCommand;command];
    diagnostics(11)=double(command==0 && previousLevel<maxRate*dtCommand);
    diagnostics(18)=hAGL;
    return
end
x0=[hAGL;vUp;previousLevel];
[minimumPeak,minValid]=predictPeak(x0,maxRate,CdLookup,Airbrake,LQR);
[maximumPeak,maxValid]=predictPeak(x0,-maxRate,CdLookup,Airbrake,LQR);
[uncontrolledPeak,uncontrolledValid]=predictPeak( ...
    [hAGL;vUp;0],0,CdLookup,Airbrake,LQR);
[trajectory,durations,count,nominalValid]=nominalRollout( ...
    x0,CdLookup,Airbrake,LQR);
heldPeak=trajectory(1,count+1);
diagnostics(3:5)=[minimumPeak;maximumPeak;heldPeak];
diagnostics(7)=heldPeak-targetAGL;
diagnostics(12)=count;
diagnostics(13)=sum(durations);
diagnostics(18)=uncontrolledPeak;
if ~(minValid && maxValid && uncontrolledValid && nominalValid) || ...
        minimumPeak>maximumPeak+LQR.ReachabilityTolerance
    % Hold rather than deploy on an unfinished or inconsistent prediction.
    diagnostics(2)=-2;
    return
end
diagnostics(1)=1;
reachable=targetAGL>=minimumPeak-LQR.ReachabilityTolerance && ...
    targetAGL<=maximumPeak+LQR.ReachabilityTolerance;
diagnostics(24)=double(reachable);
if targetAGL>=maximumPeak-LQR.ReachabilityTolerance
    rawRate=-maxRate;
    diagnostics(2)=2;
elseif targetAGL<=minimumPeak+LQR.ReachabilityTolerance
    rawRate=maxRate;
    diagnostics(2)=3;
else
    [gainInfo,riccatiValid]=affineRiccati( ...
        trajectory,durations,count,targetAGL,CdLookup,Airbrake,LQR);
    if ~riccatiValid
        diagnostics(1)=0;
        diagnostics(2)=-2;
        return
    end
    % The nominal rollout starts at the current Navigation/command state,
    % hence delta_x(0)=0. The AFFINE term is essential for target guidance.
    rawRate=-gainInfo(4);
    diagnostics(2)=1;
end
appliedRate=min(maximumRate,max(minimumRate,rawRate));
command=min(1,max(0,previousLevel+dtCommand*appliedRate));
diagnostics(8:10)=[rawRate;(command-previousLevel)/dtCommand;command];
diagnostics(11)=double(abs(rawRate-appliedRate)>1e-10);
diagnostics(14:17)=gainInfo;
end

function [gain,valid]=affineRiccati(trajectory,durations,count,target, ...
    CdLookup,Airbrake,LQR)
% Terminal velocity/deployment are unpenalized: arrival time is free and
% aerodynamic control authority vanishes as vertical speed approaches zero.
gain=zeros(4,1);
valid=false;
P=diag([1/LQR.TerminalHeightTolerance^2,0,0]);
p=P*[trajectory(1,count+1)-target;0;0];
Q=LQR.RunningWeight*diag([1/LQR.RunningHeightTolerance^2, ...
    1/LQR.RunningVelocityTolerance^2,1/LQR.RunningLevelTolerance^2]);
R=1/LQR.RateTolerance^2;
for k=count:-1:1
    dt=durations(k);
    [A,B]=discreteJacobian(trajectory(:,k),dt,CdLookup,Airbrake,LQR);
    G=dt*R+B.'*P*B;
    if ~isfinite(G) || G<=0 || any(~isfinite(A(:))) || any(~isfinite(B))
        return
    end
    K=(B.'*P*A)/G;
    affine=(B.'*p)/G;
    nextP=dt*Q+A.'*P*A-K.'*G*K;
    nextp=A.'*p-K.'*G*affine;
    P=0.5*(nextP+nextP.');
    p=nextp;
    if any(~isfinite(P(:))) || any(~isfinite(p)), return; end
    if k==1, gain=[K.';affine]; end
end
valid=all(isfinite(gain));
end

function [A,B]=discreteJacobian(x,dt,CdLookup,Airbrake,LQR)
% Secants of the actual midpoint prediction map; one-sided differences at
% deployment bounds preserve the inward rate authority at levels zero/one.
A=zeros(3,3);
for j=1:3
    xp=x;
    xm=x;
    xp(j)=x(j)+LQR.StateJacobianStep(j);
    xm(j)=x(j)-LQR.StateJacobianStep(j);
    if j==3
        xp(j)=min(1,xp(j));
        xm(j)=max(0,xm(j));
    end
    A(:,j)=(midpointStep(xp,0,dt,CdLookup,Airbrake)- ...
        midpointStep(xm,0,dt,CdLookup,Airbrake))/(xp(j)-xm(j));
end
ratePlus=min([LQR.RateJacobianStep,LQR.MaxDeploymentRate,(1-x(3))/dt]);
rateMinus=max([-LQR.RateJacobianStep,-LQR.MaxDeploymentRate,-x(3)/dt]);
B=(midpointStep(x,ratePlus,dt,CdLookup,Airbrake)- ...
    midpointStep(x,rateMinus,dt,CdLookup,Airbrake))/(ratePlus-rateMinus);
end

function [trajectory,durations,count,valid]=nominalRollout( ...
    x,CdLookup,Airbrake,LQR)
% Fixed allocation bounds keep helper memory predictable for code generation.
trajectory=zeros(3,601);
durations=zeros(1,600);
trajectory(:,1)=x;
count=0;
valid=false;
for k=1:LQR.MaxSteps
    [next,dt,apogee]=coastStep(x,0,LQR.PredictionStep,CdLookup,Airbrake);
    count=k;
    trajectory(:,k+1)=next;
    durations(k)=dt;
    if any(~isfinite(next)), return; end
    if apogee
        valid=true;
        return
    end
    x=next;
end
end

function [peak,valid]=predictPeak(x,requestedRate,CdLookup,Airbrake,LQR)
% Constant-rate full opening/retraction stops at the physical position bound;
% zero rate forecasts a fixed deployment, not an unavailable hover condition.
peak=x(1);
valid=false;
for k=1:LQR.MaxSteps
    [next,~,apogee]=coastStep(x,requestedRate, ...
        LQR.PredictionStep,CdLookup,Airbrake);
    peak=next(1);
    if any(~isfinite(next)), return; end
    if apogee
        valid=true;
        return
    end
    x=next;
end
end

function [next,usedDt,apogee]=coastStep(x,requestedRate,dt,CdLookup,Airbrake)
% Locate the vertical-velocity event within the final integration step.
next=midpointStep(x,requestedRate,dt,CdLookup,Airbrake);
usedDt=dt;
apogee=next(2)<=0;
if ~apogee, return; end
lower=0;
upper=dt;
for j=1:12
    middle=0.5*(lower+upper);
    trial=midpointStep(x,requestedRate,middle,CdLookup,Airbrake);
    if trial(2)>0, lower=middle; else, upper=middle; end
end
usedDt=0.5*(lower+upper);
next=midpointStep(x,requestedRate,usedDt,CdLookup,Airbrake);
next(2)=0;
end

function next=midpointStep(x,requestedRate,dt,CdLookup,Airbrake)
% The input is a deployment rate with bounded physical position; the
% midpoint uses the deployment present halfway through the command interval.
rate=min((1-x(3))/dt,max(-x(3)/dt,requestedRate));
a=coastAcceleration(x(1),x(2),x(3),CdLookup,Airbrake);
midH=x(1)+0.5*dt*x(2);
midV=x(2)+0.5*dt*a;
midLevel=min(1,max(0,x(3)+0.5*dt*rate));
next=[x(1)+dt*midV; ...
    x(2)+dt*coastAcceleration(midH,midV,midLevel,CdLookup,Airbrake); ...
    min(1,max(0,x(3)+dt*rate))];
end

function a=coastAcceleration(h,v,level,CdLookup,Airbrake)
altitudeMSL=Airbrake.LaunchAltitudeMSL+h;
[rho,~,soundSpeed]=rocketPlantAtmosphere(altitudeMSL);
% validateSettings validates this unchanged calibration once per guidance
% update. Revalidating it in every finite-difference/Riccati stage is redundant.
cd=validatedCd(abs(v),min(1,max(0,level)),CdLookup,soundSpeed);
if isfield(CdLookup,'Mode') && isfield(CdLookup,'Samples')
    % The validated Mode=1 schema contains Samples. Mode=2 already supplies
    % total rocket Cd, including nominal drag at zero deployment.
    cd=cd+rocketPlantInterpolate(Airbrake.NominalMach, ...
        Airbrake.NominalCdCoast,abs(v)/soundSpeed);
end
g=9.80665*(6371000/(6371000+altitudeMSL))^2;
a=-g-0.5*rho*Airbrake.ReferenceArea/Airbrake.DryMass*cd*v*abs(v);
end

function cd=validatedCd(speed,level,lookup,soundSpeed)
% Same all-point Shepard/bilinear calibration arithmetic as the public
% validated lookup, for internal finite scalar queries of already-checked data.
if isfield(lookup,'Mode') && isfield(lookup,'Samples')
    cd=0;
    if level==0,return;end
    samples=double(lookup.Samples);
    distance=hypot(samples(:,1)-level,samples(:,2)-speed/soundSpeed);
    nearest=min(distance);
    match=find(distance==0,1);
    if ~isempty(match),cd=samples(match(1),3);return;end
    weights=(nearest./distance).^3;
    cd=sum(samples(:,3).*weights)/sum(weights);
elseif isfield(lookup,'Mode') && isfield(lookup,'MachBreakpoints')
    % Structural field guards keep fixed Mode=2 structs independent of the
    % Samples/VelocityBreakpoints fields used by the other lookup schemas.
    mach=double(lookup.MachBreakpoints);
    control=double(lookup.ControlBreakpoints);
    table=double(lookup.Table);
    cd=gridCd(speed/soundSpeed,level,mach,control,table);
else
    velocity=double(lookup.VelocityBreakpoints);
    control=double(lookup.ControlBreakpoints);
    table=double(lookup.Table);
    cd=gridCd(speed,level,velocity,control,table);
end
end

function cd=gridCd(coordinate,level,axis,control,table)
coordinate=min(axis(end),max(axis(1),coordinate));
level=min(control(end),max(control(1),level));
[i,a]=lookupInterval(axis,coordinate);
[j,b]=lookupInterval(control,level);
low=table(i,j)+b*(table(i,j+1)-table(i,j));
high=table(i+1,j)+b*(table(i+1,j+1)-table(i+1,j));
cd=low+a*(high-low);
end

function [index,fraction]=lookupInterval(axis,value)
index=1;
for k=1:numel(axis)-1
    index=k;
    if value<=axis(k+1),break;end
end
fraction=(value-axis(index))/(axis(index+1)-axis(index));
end

function validateSettings(Airbrake,LQR,CdLookup)
rocketPlantValidateCdLookup(CdLookup);
assert(isstruct(Airbrake) && isscalar(Airbrake) && ...
    isfield(Airbrake,'DryMass') && isfield(Airbrake,'ReferenceArea') && ...
    isfield(Airbrake,'LaunchAltitudeMSL'), ...
    'AirbrakeLQR:InvalidOnboardModel','A scalar onboard model with mass, area and launch elevation is required.');
assert(positiveScalar(Airbrake.DryMass) && ...
    positiveScalar(Airbrake.ReferenceArea) && ...
    finiteScalar(Airbrake.LaunchAltitudeMSL), ...
    'AirbrakeLQR:InvalidOnboardModel','Finite positive mass/area and surveyed launch elevation required.');
if isfield(CdLookup,'Mode') && isfield(CdLookup,'Samples')
    assert(isfield(Airbrake,'NominalMach') && isfield(Airbrake,'NominalCdCoast'), ...
        'AirbrakeLQR:MissingNominalDrag','Incremental calibration requires nominal coast drag.');
    mach=Airbrake.NominalMach;
    nominalCd=Airbrake.NominalCdCoast;
    assert(isnumeric(mach) && isreal(mach) && isvector(mach) && ...
        isnumeric(nominalCd) && isreal(nominalCd) && isvector(nominalCd) && ...
        numel(mach)>=2 && numel(mach)==numel(nominalCd) && ...
        all(isfinite(mach(:))) && all(isfinite(nominalCd(:))) && ...
        all(mach(:)>=0) && all(diff(double(mach(:)))>0) && ...
        all(nominalCd(:)>=0), ...
        'AirbrakeLQR:InvalidNominalDrag', ...
        'Nominal Mach/Cd must be equal-length finite real vectors, increasing nonnegative Mach and nonnegative Cd.');
end
assert(isstruct(LQR) && isscalar(LQR) && ...
    isfield(LQR,'ControllerPeriod') && isfield(LQR,'PredictionStep') && ...
    isfield(LQR,'MaxSteps') && isfield(LQR,'MaxDeploymentRate') && ...
    isfield(LQR,'TerminalHeightTolerance') && ...
    isfield(LQR,'RunningHeightTolerance') && ...
    isfield(LQR,'RunningVelocityTolerance') && ...
    isfield(LQR,'RunningLevelTolerance') && isfield(LQR,'RateTolerance') && ...
    isfield(LQR,'StateJacobianStep') && isfield(LQR,'RateJacobianStep') && ...
    isfield(LQR,'RunningWeight') && isfield(LQR,'ReachabilityTolerance'), ...
    'AirbrakeLQR:InvalidSettings','A scalar complete LQR settings structure is required.');
assert(positiveScalar(LQR.ControllerPeriod) && ...
    positiveScalar(LQR.PredictionStep) && positiveScalar(LQR.MaxSteps) && ...
    positiveScalar(LQR.MaxDeploymentRate) && ...
    positiveScalar(LQR.TerminalHeightTolerance) && ...
    positiveScalar(LQR.RunningHeightTolerance) && ...
    positiveScalar(LQR.RunningVelocityTolerance) && ...
    positiveScalar(LQR.RunningLevelTolerance) && ...
    positiveScalar(LQR.RateTolerance) && positiveScalar(LQR.RateJacobianStep) && ...
    LQR.MaxSteps==round(LQR.MaxSteps) && LQR.MaxSteps<=600 && ...
    isnumeric(LQR.StateJacobianStep) && isreal(LQR.StateJacobianStep) && ...
    isequal(size(LQR.StateJacobianStep),[3,1]) && ...
    all(isfinite(LQR.StateJacobianStep)) && all(LQR.StateJacobianStep>0) && ...
    finiteScalar(LQR.RunningWeight) && LQR.RunningWeight>=0 && ...
    finiteScalar(LQR.ReachabilityTolerance) && LQR.ReachabilityTolerance>=0, ...
    'AirbrakeLQR:InvalidSettings','Positive finite scales and an integer horizon of at most 600 steps required.');
end

function valid=finiteScalar(value)
valid=isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end

function valid=positiveScalar(value)
valid=finiteScalar(value) && value>0;
end
