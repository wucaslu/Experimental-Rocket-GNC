function xNext = rocketPlantStep(x,t,g,Rocket,Weather,gustNED)
%ROCKETPLANTSTEP Explicit-state RK4 rocket dynamics and deterministic events.
% Advances Rocket.Ts from time t; substeps do not exceed Rocket.MaxStep.
% Events are localized by linear crossing interpolation (within a substep).
% Phase: 0 pad, 1 launch rail, 2 free, 3 drogue, 4 main, 5 landed.
% No persistent variables, randomness, file input or hidden object state.
% Parachutes apply forces with finite inflation; velocity remains continuous
% at deployment. Ground impact is the only velocity reset.
    if nargin<5, Weather=[]; end
    % Match the Environment driver: sample once per Rocket.Ts, then hold during
    % the RK4 substeps. Altitude-dependent weather is still evaluated locally.
    if nargin<6, gustNED=rocketWeatherGust(t,Weather); end
    xNext=x;
    initial=rocketPlantInitialize(Rocket);
    launchR=rocketPlantRotation(initial(7:10));
    rail=launchR(:,3);
    n=max(1,ceil(Rocket.Ts/max(Rocket.MaxStep,eps)));
    dt=Rocket.Ts/n;
    for step=1:n
        time=t+(step-1)*dt;
        if xNext(14)<0.5
            [thrust,mass]=rocketPlantProperties(time,Rocket);
            direction=launchR*(Rocket.ThrustDirection/max(norm(Rocket.ThrustDirection),eps));
            if time>=Rocket.IgnitionTime && thrust*dot(direction,rail)+mass*g*rail(3)>0
                xNext(14)=1;
            end
        end
        xNext=activateDeployments(xNext,time);
        before=xNext;
        k1=rocketPlantDerivative(before,time,g,Rocket,Weather,gustNED);
        k2=rocketPlantDerivative(before+0.5*dt*k1,time+0.5*dt,g,Rocket,Weather,gustNED);
        k3=rocketPlantDerivative(before+0.5*dt*k2,time+0.5*dt,g,Rocket,Weather,gustNED);
        k4=rocketPlantDerivative(before+dt*k3,time+dt,g,Rocket,Weather,gustNED);
        xNext=before+dt*(k1+2*k2+2*k3+k4)/6;
        xNext(7:10)=xNext(7:10)/max(norm(xNext(7:10)),eps);
        endTime=time+dt;
        if xNext(14)>0.5 && xNext(14)<1.5
            distance=max(0,dot(xNext(1:3),rail));
            alongSpeed=dot(xNext(4:6),rail);
            if distance<=0
                alongSpeed=max(0,alongSpeed);
            end
            xNext(1:3)=rail*distance;
            xNext(4:6)=rail*alongSpeed;
            xNext(7:10)=initial(7:10);
            xNext(11:13)=zeros(3,1);
            if distance>=Rocket.RailLength
                xNext(14)=2;
            end
        end
        if xNext(14)>1.5 && xNext(14)<4.5
            % Record apogee when vertical NED velocity first crosses zero.
            if xNext(15)<0 && before(6)<0 && xNext(6)>=0
                fraction=max(0,min(1,-before(6)/max(xNext(6)-before(6),eps)));
                xNext(15)=time+fraction*dt;
                xNext(16)=xNext(15)+Rocket.DrogueLag;
            end
            % Main threshold only while descending, with independent latency.
            if xNext(15)>=0 && xNext(17)<0 && xNext(6)>0 && -xNext(3)<=Rocket.MainAltitudeAGL
                fraction=1;
                if -before(3)>Rocket.MainAltitudeAGL
                    fraction=max(0,min(1,(-before(3)-Rocket.MainAltitudeAGL)/ ...
                        max(xNext(3)-before(3),eps)));
                end
                xNext(17)=time+fraction*dt+Rocket.MainLag;
            end
            xNext=activateDeployments(xNext,endTime);
            % Plane ground contact; retain horizontal impact point and attitude.
            if xNext(3)>=0 && xNext(6)>0
                fraction=max(0,min(1,-before(3)/max(xNext(3)-before(3),eps)));
                xNext(1:2)=before(1:2)+fraction*(xNext(1:2)-before(1:2));
                xNext(3)=0;
                xNext(4:6)=zeros(3,1);
                xNext(11:13)=zeros(3,1);
                xNext(14)=5;
            end
        end
    end
end

function state=activateDeployments(state,time)
    if state(14)>1.5 && state(14)<4.5
        if state(17)>=0 && time>=state(17)
            state(14)=4;
        elseif state(16)>=0 && time>=state(16)
            state(14)=3;
        end
    end
end
