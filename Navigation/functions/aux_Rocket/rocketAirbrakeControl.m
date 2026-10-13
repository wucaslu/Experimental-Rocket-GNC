function [level,lookupEnabled,nextState,info] = rocketAirbrakeControl( ...
    positionNEU,velocityNEU,qNB,accMeasuredBody,accBiasBody, ...
    globalEnable,targetAGL,state,CdLookup,Airbrake)
%ROCKETAIRBRAKECONTROL Explicit-state Navigation/IMU airbrake reference.
% Inputs: positionNEU [3x1] m = estimated local North/East and absolute MSL
% Up; velocityNEU [3x1] m/s = estimated ground North/East/Up; qNB [4x1] is
% scalar-last active Hamilton body-to-NED attitude. accMeasuredBody and
% accBiasBody [3x1] m/s^2 are measured specific force and estimated bias.
% globalEnable is the overall scalar switch; targetAGL [m] is the positive
% apogee target used by ControlLawMode=0, the proportional apogee predictor.
% ControlLawMode=1 implements the RocketPy documentation's intended velocity
% law using the previous CONTROLLER sample's upward velocity. Deployment is
% continuous and held between 10 Hz updates, with no second actuator slew,
% speed gate or descent-stop latch. Its scheduled 1500 m altitude decision
% and velocity gains do not explicitly regulate targetAGL. Mode 2 retains the
% earlier constrained implementation: 200 m/s gate, descent stop, quantized
% commands and separate physical actuator slew. The published
% Python example's time-prefixed history indexing differs from this intent.
% CdLookup and Airbrake contain only independent onboard calibration/settings.
% Explicit state [10x1] = [actual level; ascentSeen; boostSeen; coastLatched;
% stopLatched; global sample counter; held forecast AGL; commanded deployment;
% continuous request; previous controller-sample upward velocity]. An old
% eight-element state is accepted with the new two states initially zero.
% Outputs: level is physical deployment (equal to the command in mode 1,
% independently rate limited in modes 0 and 2); nextState
% always has ten elements. lookupEnabled is always true. info [12x1] =
% [height AGL; upward speed; coast; ascent; stop; gate; held forecast AGL;
% target AGL; forecast error; commanded deployment; actual level; table active].
% In modes 1 and 2 the forecast is retained without an apogee prediction.
% Native Simulink stores flight flags, guidance state and root Actuator state
% in separate Unit Delays. This numerical reference combines their behavior.
% In modes 1 and 2 the counter advances even while disabled; velocity memory is
% updated only on scheduled controller samples. Mode 0 retains its descending
% forecast counter, reset on disable. No persistent variables or timers
% decide launch, burnout or descent permission. No truth/wind/GNSS velocity.
assert((numel(state)==8 || numel(state)==10) && all(isfinite(state)) && ...
    isfinite(targetAGL) && targetAGL>0, ...
    'Airbrake:InvalidState','Finite explicit state and positive AGL target required.');
memory=zeros(10,1);
memory(1:numel(state))=state(:);
mode=0;
if isfield(Airbrake,'ControlLawMode'), mode=Airbrake.ControlLawMode; end
assert(isscalar(mode) && isfinite(mode) && (mode==0 || mode==1 || mode==2), ...
    'Airbrake:InvalidControlLawMode','ControlLawMode must be numeric zero, one or two.');
period=Airbrake.Ts*Airbrake.PredictionEvery;
if mode~=0 && isfield(Airbrake,'ControllerPeriod'), period=Airbrake.ControllerPeriod; end
samples=round(period/Airbrake.Ts);
assert(Airbrake.Ts>0 && isfinite(period) && samples>=1 && ...
    abs(samples*Airbrake.Ts-period)<=1e-10*max(1,period), ...
    'Airbrake:InvalidControllerPeriod','Controller period must be an integer sample multiple.');
counter=memory(6);
assert(counter==round(counter) && counter>=0 && counter<samples, ...
    'Airbrake:InvalidCounter','Controller counter must be on the explicit sample grid.');
sampleNow=counter==0;
nextCounter=mod(counter+1,samples);
h=positionNEU(3)-Airbrake.LaunchAltitudeMSL;
v=velocityNEU(3);
qNorm=norm(qNB);
feedbackValid=all(isfinite([positionNEU(:);velocityNEU(:);qNB(:); ...
    accMeasuredBody(:);accBiasBody(:)])) && qNorm>1e-12;
ascentSeen=memory(2)>0.5 || (feedbackValid && ...
    h>=Airbrake.ArmHeightAGL && v>=Airbrake.ArmUpVelocity);
stopped=memory(5)>0.5 || (feedbackValid && ascentSeen && v<=-Airbrake.StopDownSpeed);
if mode==1, stopped=false; end
forceProjectionNeeded=feedbackValid && ascentSeen && memory(4)<=0.5 && ~stopped;
fUp=0;
if forceProjectionNeeded
    specificForceNED=quatToDcm(qNB(:)/qNorm)*(accMeasuredBody(:)-accBiasBody(:));
    fUp=-specificForceNED(3);
end
boostSeen=memory(3)>0.5 || (forceProjectionNeeded && ...
    fUp>=Airbrake.BoostSpecificForceUp);
coastLatched=memory(4)>0.5 || (forceProjectionNeeded && boostSeen && ...
    v>=Airbrake.ArmUpVelocity && fUp<=Airbrake.CoastSpecificForceUp);
gate=isfinite(globalEnable) && globalEnable~=0 && feedbackValid && ...
    ascentSeen && coastLatched && v<Airbrake.MaxUpVelocity && ~stopped;
if mode==1
    gate=isfinite(globalEnable) && globalEnable~=0 && feedbackValid && coastLatched;
elseif mode==2
    gate=gate && h>=Airbrake.DeploymentAltitudeAGL;
end
forecast=memory(7);
desired=memory(8);
request=memory(9);
if mode==1, request=desired; end
previousUp=memory(10);
if ~gate
    desired=0;
    request=0;
    if mode==0, nextCounter=0; end
elseif sampleNow
    if mode==1 || mode==2
        % The example bounds deployment changes at controller ticks. Mode 2
        % accumulates these increments before its .1 command quantization.
        if mode==1 && h<Airbrake.DeploymentAltitudeAGL
            % The example sets zero at a controller tick below 1500 m AGL.
            % This bypasses its incremental rate limit, including on descent.
            request=0;
        else
            increment=0.1*v+0.01*previousUp^2;
            maxChange=Airbrake.DeploymentRate*period;
            request=min(1,max(0,request+min(maxChange,max(-maxChange,increment))));
        end
    else
        [forecast,predictionValid]=rocketAirbrakePredictApogee(max(0,h),v,CdLookup,Airbrake);
        request=0;
        if predictionValid
            request=min(1,max(0,Airbrake.ApogeeGain*(forecast-targetAGL)));
        end
    end
    if mode==1
        desired=request;
    else
        commandBins=round(1/Airbrake.CommandStep);
        desired=min(1,max(0,round(request*commandBins)/commandBins));
    end
    if mode==0
        nextCounter=samples-1;
        request=desired;
    end
elseif mode==0
    nextCounter=counter-1;
    request=desired;
end
if sampleNow && isfinite(v)
    previousUp=v;
end
if mode==1
    level=desired;
else
    step=Airbrake.MaxLevelRate*Airbrake.Ts;
    level=min(1,max(0,memory(1)+min(step,max(-step,desired-memory(1)))));
end
lookupEnabled=true;
nextState=[level;double(ascentSeen);double(boostSeen);double(coastLatched); ...
    double(stopped);nextCounter;forecast;desired;request;previousUp];
info=[h;v;double(coastLatched);double(ascentSeen);double(stopped);double(gate); ...
    forecast;targetAGL;forecast-targetAGL;desired;level;double(lookupEnabled)];
end
