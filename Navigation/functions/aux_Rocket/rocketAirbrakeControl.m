function [level,lookupEnabled,nextState,info] = rocketAirbrakeControl( ...
    positionNEU,velocityNEU,qNB,accMeasuredBody,accBiasBody, ...
    globalEnable,targetAGL,state,CdLookup,Airbrake)
%ROCKETAIRBRAKECONTROL Navigation/IMU-based proportional coast guidance.
% Inputs: positionNEU [3x1] m = estimated local North/East and absolute MSL
% Up; velocityNEU [3x1] m/s = estimated ground North/East/Up; qNB [4x1] is
% estimated active Hamilton body-to-NED quaternion [x;y;z;w].
% accMeasuredBody and accBiasBody [3x1] m/s^2 are measured specific force and
% estimated accelerometer bias. globalEnable is the overall scalar switch;
% targetAGL is the positive apogee target above launch [m]. CdLookup contains
% the onboard Cd table; Airbrake contains independent nominal vehicle/site
% constants, detector thresholds, guidance weights and actuator settings.
% Explicit state [8x1] = [actual level; ascentSeen; boostSeen; coastLatched;
% stopLatched; forecast counter; held apogee AGL; quantized target level].
% Outputs: level is continuous rate-limited deployment; lookupEnabled is
% always true; nextState retains the eight-state layout. info [12x1] =
% [height AGL; upward speed; detected coast; ascentSeen; stopLatched; gate;
% forecast AGL; target AGL; forecast error; target level; actual level; 1].
% Native Simulink stores state(2:5) in Flight_enable. Within Control Law,
% Guidance_enabled conditionally computes forecast/command; the always-running
% Actuator_and_diagnostics maintains state([1 6 7 8]) and the current log.
% This reference combines their behavior for independent numeric checks.
% No true flight states, motor status, weather, GNSS velocity or timers.
assert(numel(state)==8 && all(isfinite(state)) && isfinite(targetAGL) && targetAGL>0, ...
    'Airbrake:InvalidState','Finite explicit state and positive AGL target required.');
h=positionNEU(3)-Airbrake.LaunchAltitudeMSL;
v=velocityNEU(3);
qNorm=norm(qNB);
feedbackValid=all(isfinite([positionNEU(:);velocityNEU(:);qNB(:); ...
    accMeasuredBody(:);accBiasBody(:)])) && qNorm>1e-12;
ascentSeen=state(2)>0.5 || (feedbackValid && ...
    h>=Airbrake.ArmHeightAGL && v>=Airbrake.ArmUpVelocity);
stopped=state(5)>0.5 || (feedbackValid && ascentSeen && v<=-Airbrake.StopDownSpeed);
% Force projection is needed only while identifying boost and coast. Retain
% all input-validity checks so invalid feedback still closes permission.
forceProjectionNeeded=feedbackValid && ascentSeen && state(4)<=0.5 && ~stopped;
fUp=0;
if forceProjectionNeeded
    specificForceNED=quatToDcm(qNB(:)/qNorm)*(accMeasuredBody(:)-accBiasBody(:));
    fUp=-specificForceNED(3);
end
boostSeen=state(3)>0.5 || (forceProjectionNeeded && ...
    fUp>=Airbrake.BoostSpecificForceUp);
coastLatched=state(4)>0.5 || (forceProjectionNeeded && boostSeen && ...
    v>=Airbrake.ArmUpVelocity && fUp<=Airbrake.CoastSpecificForceUp);
gate=isfinite(globalEnable) && globalEnable~=0 && feedbackValid && ...
    ascentSeen && coastLatched && v<Airbrake.MaxUpVelocity && ~stopped;
forecast=state(7);
desired=0;
counter=state(6);
if gate
    if counter<=0
        [forecast,predictionValid]=rocketAirbrakePredictApogee(max(0,h),v,CdLookup,Airbrake);
        if predictionValid
            desired=min(1,max(0,Airbrake.ApogeeGain*(forecast-targetAGL)));
        end
        counter=Airbrake.PredictionEvery-1;
    else
        desired=state(8);
        counter=counter-1;
    end
else
    counter=0;
end
% Quantized actuator targets, with continuous rate-limited physical motion.
commandBins=round(1/Airbrake.CommandStep);
desired=min(1,max(0,round(desired*commandBins)/commandBins));
step=Airbrake.MaxLevelRate*Airbrake.Ts;
level=min(1,max(0,state(1)+min(step,max(-step,desired-state(1)))));
% A closed flight gate retracts the actuator; it never disables the Cd table.
lookupEnabled=true;
nextState=[level;double(ascentSeen);double(boostSeen);double(coastLatched); ...
    double(stopped);counter;forecast;desired];
info=[h;v;double(coastLatched);double(ascentSeen);double(stopped);double(gate); ...
    forecast;targetAGL;forecast-targetAGL;desired;level;double(lookupEnabled)];
end
