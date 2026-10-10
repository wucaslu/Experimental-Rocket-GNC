function [level,lookupEnabled,nextState,info] = rocketAirbrakeControl( ...
    positionNEU,velocityNEU,motorDebug,requestedLevel,requestedEnable, ...
    targetAGL,automatic,enabled,state,Rocket,Weather,CdLookup,Airbrake)
%ROCKETAIRBRAKECONTROL Proportional coast guidance with explicit flight gates.
% Navigation position is absolute MSL NEU; velocity is NEU ground velocity.
% No GNSS velocity, persistent variables, timers or hidden flight state.
% motorDebug: Environment's18-vector; indices1/2 are phase and motor mass.
assert(numel(state)==6 && all(isfinite(state)) && isfinite(targetAGL) && targetAGL>0, ...
    'Airbrake:InvalidState','Finite explicit state and positive AGL target required.');
h=positionNEU(3)-Rocket.AltitudeMSL;
v=velocityNEU(3);
feedbackValid=all(isfinite([positionNEU(:);velocityNEU(:);motorDebug(:)]));
burnedOut=feedbackValid && motorDebug(1)>0 && ...
    motorDebug(2)<=Rocket.DryMass+1e-9;
ascentSeen=state(2)>0.5 || (feedbackValid && h>=Airbrake.ArmHeightAGL && v>=Airbrake.ArmUpVelocity);
stopped=state(3)>0.5 || (ascentSeen && burnedOut && v<=-Airbrake.StopDownSpeed);
gate=enabled~=0 && feedbackValid && burnedOut && ascentSeen && ...
    v<Airbrake.MaxUpVelocity && ~stopped;
forecast=state(5);
desired=0;
counter=state(4);
if gate
    if automatic~=0
        if counter<=0
            [forecast,predictionValid]=rocketAirbrakePredictApogee(max(0,h),v,Rocket,Weather,CdLookup,Airbrake);
            if predictionValid
                desired=min(1,max(0,Airbrake.ApogeeGain*(forecast-targetAGL)));
            end
            counter=Airbrake.PredictionEvery-1;
        else
            desired=state(6);
            counter=counter-1;
        end
    elseif requestedEnable~=0 && isfinite(requestedLevel)
        desired=min(1,max(0,requestedLevel));
        counter=0;
    end
else
    counter=0;
end
% The actuator accepts eleven target positions, 0:0.1:1. Quantize the
% target before simulating its continuous, rate-limited physical motion.
commandBins=round(1/Airbrake.CommandStep);
desired=min(1,max(0,round(desired*commandBins)/commandBins));
step=Airbrake.MaxLevelRate*Rocket.Ts;
level=min(1,max(0,state(1)+min(step,max(-step,desired-state(1)))));
% Keep the physical lookup active while retracting after guidance stops.
lookupEnabled=(gate && (automatic~=0 || requestedEnable~=0)) || level>1e-12;
nextState=[level;double(ascentSeen);double(stopped);counter;forecast;desired];
% Log: h,v,burnout,armed,stop,gate,forecast,target,error,command,actual,lookup.
info=[h;v;double(burnedOut);double(ascentSeen);double(stopped);double(gate); ...
    forecast;targetAGL;forecast-targetAGL;desired;level;double(lookupEnabled)];
end
