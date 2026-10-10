function [forceB,momentB,aero] = rocketPlantAerodynamics(x,t,Rocket,Weather,gustNED)
    %ROCKETPLANTAERODYNAMICS Body axial drag and component Barrowman loads.
    % Body +Z is noseward, positions are relative to the common plant CM.
    % Nose, fins and tail each see wind at their CP height and rotational flow.
    % CM density/sound speed are used, as in RocketPy's Barrowman flight path.
    % The linear normal-force law is an approximation at large angle of attack.
    % References: RocketPy AeroSurface.compute_forces_and_moments and Fins.
    % https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/aero_surface.html
    % https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/fins/fins.html
    
    if nargin<4, Weather=[]; end
    if nargin<5, gustNED=rocketWeatherGust(t,Weather); end
    R=rocketPlantRotation(x(7:10));
    omega=x(11:13);
    hCM=-x(3);
    [windCM,rho,~,soundSpeed]=rocketWeatherConditions(Rocket.AltitudeMSL+hCM,Rocket,Weather,gustNED);
    vCM=R.'*(x(4:6)-windCM);
    speedCM=norm(vCM);
    machCM=speedCM/soundSpeed;
    dynamicPressureCM=0.5*rho*speedCM^2;
    burning=t>=Rocket.IgnitionTime && ...
        t<Rocket.IgnitionTime+Rocket.ThrustTime(end);
    if burning
        cd=rocketPlantInterpolate(Rocket.AeroMach,Rocket.AeroCdPowered,machCM);
    else
        cd=rocketPlantInterpolate(Rocket.AeroMach,Rocket.AeroCdCoast,machCM);
    end
    n=size(Rocket.AeroSurfaceCP,1);
    aero=struct('machCM',machCM,'dynamicPressureCM',dynamicPressureCM,'cd',cd, ...
        'alpha',zeros(n,1),'mach',zeros(n,1),'speed',zeros(n,1), ...
        'cna',zeros(n,1),'surfaceForce',zeros(n,3),'surfaceMoment',zeros(n,3), ...
        'cpWeighted',zeros(3,1),'staticMargin',0,'rollMoment',zeros(n,1));
    forceB=zeros(3,1);
    momentB=zeros(3,1);
    if Rocket.AeroEnabled<0.5
        return
    end
    % RocketPy's forward-flight drag model applies axial drag at the CM.
    forceB(3)=-dynamicPressureCM*Rocket.ReferenceArea*cd;
    weightedCP=zeros(3,1);
    totalSlope=0;
    for i=1:n
        cp=Rocket.AeroSurfaceCP(i,:).';
        cpN=R*cp;
        hCP=hCM-cpN(3);
        windCP=rocketWeatherWind(Rocket.AltitudeMSL+hCP,Rocket,Weather,gustNED);
        vCP=R.'*(x(4:6)-windCP)+cross(omega,cp);
        speed=norm(vCP);
        mach=speed/soundSpeed;
        [cna,singleFinSlope]=surfaceSlope(i,mach,Rocket);
        [cmSlope,~]=surfaceSlope(i,machCM,Rocket);
        weightedCP=weightedCP+cmSlope*cp;
        totalSlope=totalSlope+cmSlope;
        normalForce=zeros(3,1);
        alpha=0;
        if speed>1e-10
            alpha=acos(max(-1,min(1,vCP(3)/speed)));
            transverseSpeed=hypot(vCP(1),vCP(2));
            if transverseSpeed>1e-10
                scale=-0.5*rho*speed^2*Rocket.AeroSurfaceArea(i)*cna*alpha/transverseSpeed;
                normalForce(1:2)=scale*vCP(1:2);
            end
        end
        diameter=Rocket.AeroSurfaceDiameter(i);
        rollForceCoefficient=Rocket.AeroRollForceScale(i)*singleFinSlope;
        rollDampCoefficient=Rocket.AeroRollDampScale(i)*singleFinSlope;
        rollMoment=0.5*rho*speed^2*Rocket.AeroSurfaceArea(i)*diameter* ...
            rollForceCoefficient*Rocket.AeroSurfaceCant(i)- ...
            0.25*rho*speed*Rocket.AeroSurfaceArea(i)*diameter^2* ...
            rollDampCoefficient*omega(3);
        surfaceMoment=cross(cp,normalForce)+[0;0;rollMoment];
        forceB=forceB+normalForce;
        momentB=momentB+surfaceMoment;
        aero.alpha(i)=alpha;
        aero.mach(i)=mach;
        aero.speed(i)=speed;
        aero.cna(i)=cna;
        aero.surfaceForce(i,:)=normalForce.';
        aero.surfaceMoment(i,:)=surfaceMoment.';
        aero.rollMoment(i)=rollMoment;
    end
    if abs(totalSlope)>1e-10
        aero.cpWeighted=weightedCP/totalSlope;
        aero.staticMargin=-aero.cpWeighted(3)/(2*Rocket.BodyRadius);
    end
end

function [cna,singleFinSlope]=surfaceSlope(i,mach,Rocket)
    singleFinSlope=0;
    if Rocket.AeroFinPlanform(i)>0
        % RocketPy's transonic compressibility plateau avoids the Mach-1 pole.
        if mach<0.8
            beta=sqrt(1-mach^2);
        elseif mach<1.1
            beta=0.6;
        else
            beta=sqrt(mach^2-1);
        end
        singleFinSlope=Rocket.AeroFinLiftNumerator(i)/ ...
            (2+sqrt((beta*Rocket.AeroFinPlanform(i))^2+4));
        cna=singleFinSlope*Rocket.AeroFinLiftMultiplier(i);
    else
        cna=Rocket.AeroSurfaceCNa(i);
    end
end
