function [aN,alphaB,dbg] = rocketPlantLoads(x,t,g,Rocket)
%ROCKETPLANTLOADS Translational and rotational truth, including constraints.
% Fixed-CM diagonal Euler equation includes approximate exhaust angular
% flux: I*alpha = M - omega x (I*omega) + (mdot*Snozzle-Idot)*omega.
% NozzleGyration is the diagonal exhaust-stream gyration tensor [m^2].
% Aero force acts at a fixed body CP; angular damping is dimensional [Nm s].
% Chute forces use attachment air velocity and smooth finite inflation;
% their deployment changes force continuously and never resets velocity.
% No Coriolis/curvature, moving-CM coupling, flexible bodies, canopy swing,
% turbulent wind or atmospheric uncertainty is included.
    R=rocketPlantRotation(x(7:10));
    w=x(11:13);
    hAGL=-x(3);
    [rho,pressure,soundSpeed]=rocketPlantAtmosphere(Rocket.AltitudeMSL+hAGL);
    [thrust,mass,I,massRate,inertiaRate]=rocketPlantProperties(t,Rocket);
    windN=zeros(3,1);
    for i=1:3
        windN(i)=rocketPlantInterpolate(Rocket.WindAltitude,Rocket.WindNED(:,i),hAGL);
    end
    vAirB=R.'*(x(4:6)-windN);
    vCP=vAirB+cross(w,Rocket.AeroCP);
    speedCP=norm(vCP);
    mach=speedCP/soundSpeed;
    if thrust>0
        cd=rocketPlantInterpolate(Rocket.AeroMach,Rocket.AeroCdPowered,mach);
    else
        cd=rocketPlantInterpolate(Rocket.AeroMach,Rocket.AeroCdCoast,mach);
    end
    dynamicPressure=0.5*rho*speedCP*speedCP;
    fAero=-0.5*rho*Rocket.ReferenceArea*cd*speedCP*vCP;
    transverse=[vCP(1);vCP(2);0];
    transverseSpeed=norm(transverse);
    if transverseSpeed>1e-10
        alpha=atan2(transverseSpeed,abs(vCP(3)));
        fAero=fAero-dynamicPressure*Rocket.ReferenceArea*Rocket.CNa*alpha* ...
            (transverse/transverseSpeed);
    end
    thrustDirection=Rocket.ThrustDirection/max(norm(Rocket.ThrustDirection),eps);
    fThrust=thrust*thrustDirection;
    forceB=fThrust+fAero;
    momentB=cross(Rocket.ThrustOffset,fThrust)+cross(Rocket.AeroCP,fAero)- ...
        Rocket.AngularDamping.*w;
    drogueCdS=inflatedCdS(t,x(16),Rocket.DrogueInflationTime,Rocket.DrogueCdS);
    mainCdS=inflatedCdS(t,x(17),Rocket.MainInflationTime,Rocket.MainCdS);
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
