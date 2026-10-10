function [thrust,mass,inertia,massRate,inertiaRate] = rocketPlantProperties(t,Rocket)
%ROCKETPLANTPROPERTIES Motor mass and inertia from exact linear thrust impulse.
% Propellant flow is proportional to thrust (constant effective exhaust
% velocity). The normalized cumulative impulse specifies knot values. Dry
% and propellant inertias are diagonal and referenced to a coincident fixed
% CM; movement of CM and parallel-axis terms are deliberately approximated.
tau=t-Rocket.IgnitionTime;
n=numel(Rocket.ThrustTime);
totalImpulse=0;
for i=1:n-1
    totalImpulse=totalImpulse+0.5*(Rocket.ThrustForce(i)+Rocket.ThrustForce(i+1))* ...
        (Rocket.ThrustTime(i+1)-Rocket.ThrustTime(i));
end
thrust=0;
fraction=1;
massRate=0;
if tau>=Rocket.ThrustTime(n)
    fraction=0;
elseif tau>=Rocket.ThrustTime(1)
    k=1;
    for i=1:n-1
        if tau>=Rocket.ThrustTime(i) && tau<Rocket.ThrustTime(i+1)
            k=i;
            break
        end
    end
    dt=tau-Rocket.ThrustTime(k);
    slope=(Rocket.ThrustForce(k+1)-Rocket.ThrustForce(k))/ ...
        (Rocket.ThrustTime(k+1)-Rocket.ThrustTime(k));
    thrust=max(0,Rocket.ThrustForce(k)+slope*dt);
    segmentImpulse=Rocket.ThrustForce(k)*dt+0.5*slope*dt*dt;
    usedImpulse=Rocket.ThrustCumulativeImpulse(k)+segmentImpulse/max(totalImpulse,eps);
    fraction=max(0,min(1,1-usedImpulse));
    massRate=-Rocket.PropellantMass*thrust/max(totalImpulse,eps);
end
mass=Rocket.DryMass+fraction*Rocket.PropellantMass;
inertia=Rocket.DryInertia+fraction*Rocket.PropellantInertia;
inertiaRate=Rocket.PropellantInertia*(massRate/max(Rocket.PropellantMass,eps));
end
