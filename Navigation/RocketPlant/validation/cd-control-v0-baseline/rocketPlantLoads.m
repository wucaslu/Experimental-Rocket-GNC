function [aN,alphaB,dbg] = rocketPlantLoads(x,t,g,Rocket,Weather,gustNED)
%ROCKETPLANTLOADS Translational and rotational truth, including constraints.
% Fixed-CM diagonal Euler equation includes approximate exhaust angular
% flux: I*alpha = M - omega x (I*omega) + (mdot*Snozzle-Idot)*omega.
% NozzleGyration is the diagonal exhaust-stream gyration tensor [m^2].
% Nose, fins and tail supply separate Barrowman forces and moments.
% AngularDamping is optional residual dimensional damping [Nm s].
% Chute forces use attachment air velocity and smooth finite inflation;
% their deployment changes force continuously and never resets velocity.
% Earth rotation, moving-CM coupling and canopy swing are omitted.
% Weather is an explicit
% configuration and gust signal, held during each major integration step.

    if nargin<5, Weather=[]; end
    if nargin<6, gustNED=rocketWeatherGust(t,Weather); end
    R=rocketPlantRotation(x(7:10));
    w=x(11:13);
    hAGL=-x(3);
    [windN,rho,pressure,soundSpeed]=rocketWeatherConditions(Rocket.AltitudeMSL+hAGL,Rocket,Weather,gustNED);
    [thrust,mass,I,massRate,inertiaRate]=rocketPlantProperties(t,Rocket);
    vAirB=R.'*(x(4:6)-windN);
    [fAero,mAero]=rocketPlantAerodynamics(x,t,Rocket,Weather,gustNED);
    mach=norm(vAirB)/soundSpeed;
    thrustDirection=Rocket.ThrustDirection/max(norm(Rocket.ThrustDirection),eps);
    fThrust=thrust*thrustDirection;
    drogueCdS=inflatedCdS(t,x(16),Rocket.DrogueInflationTime,Rocket.DrogueCdS);
    mainCdS=inflatedCdS(t,x(17),Rocket.MainInflationTime,Rocket.MainCdS);
    % RocketPy switches to parachute descent dynamics. This plant retains 6DOF
    % and smoothly removes the small-angle body law as a canopy inflates.
    recoveryFraction=0;
    if Rocket.DrogueCdS>0
        recoveryFraction=max(recoveryFraction,drogueCdS/Rocket.DrogueCdS);
    end
    if Rocket.MainCdS>0
        recoveryFraction=max(recoveryFraction,mainCdS/Rocket.MainCdS);
    end
    bodyAeroFraction=1-min(1,recoveryFraction);
    forceB=fThrust+bodyAeroFraction*fAero;
    momentB=cross(Rocket.ThrustOffset,fThrust)+bodyAeroFraction*mAero- ...
        Rocket.AngularDamping.*w;
    vDrogue=vAirB+cross(w,Rocket.DrogueAttachment);
    fDrogue=-0.5*rho*drogueCdS*norm(vDrogue)*vDrogue;
    vMain=vAirB+cross(w,Rocket.MainAttachment);
    fMain=-0.5*rho*mainCdS*norm(vMain)*vMain;
    forceB=forceB+fDrogue+fMain;
    momentB=momentB+cross(Rocket.DrogueAttachment,fDrogue)+ ...
        cross(Rocket.MainAttachment,fMain);
    aN=R*forceB/mass+[0;0;g];
    alphaB=(momentB-cross(w,I.*w)+(massRate*Rocket.NozzleGyration-inertiaRate).*w)./I;
    phase=x(14);
    if phase<0.5 || phase>4.5
        aN=zeros(3,1);
        alphaB=zeros(3,1);
    elseif phase<1.5
        initial=rocketPlantInitialize(Rocket);
        launchR=rocketPlantRotation(initial(7:10));
        rail=launchR(:,3);
        alongAcceleration=dot(aN,rail);
        if dot(x(1:3),rail)<=0 && dot(x(4:6),rail)<=0
            alongAcceleration=max(0,alongAcceleration);
        end
        aN=rail*alongAcceleration;
        alphaB=zeros(3,1);
    end
    % phase,mass,thrust,groundSpeed,airSpeed,Mach,rho,pressure,drogueCdS,
    % mainCdS,AGL,apogeeTime,drogueTime,mainTime,aN(3),norm(alphaB).
    dbg=[phase;mass;thrust;norm(x(4:6));norm(vAirB);mach;rho;pressure; ...
        drogueCdS;mainCdS;hAGL;x(15:17);aN;norm(alphaB)];
end

function cdS=inflatedCdS(t,deployTime,inflationTime,fullCdS)
    cdS=0;
    if deployTime>=0 && t>=deployTime
        s=max(0,min(1,(t-deployTime)/max(inflationTime,eps)));
        cdS=fullCdS*s*s*(3-2*s);
    end
end
