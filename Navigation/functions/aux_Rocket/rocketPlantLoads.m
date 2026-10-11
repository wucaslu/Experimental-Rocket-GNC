function [aN,alphaB,dbg,cdInfo] = rocketPlantLoads(x,t,g,Rocket,Weather,gustNED,cdCommand,cdOverrideEnabled)
%ROCKETPLANTLOADS Translational and rotational truth, including constraints.
% States translate a fixed body reference O; the instantaneous CM changes
% with propellant depletion. Coupled fixed-origin variable-mass equations
% include CM velocity/acceleration, parallel-axis inertia and exhaust flux.
% NozzleGyration is the existing approximate tensor about fixed O [m^2].
% Nose, fins and tail supply separate Barrowman forces and moments.
% AngularDamping is optional residual dimensional damping [Nm s].
% Chute forces use attachment air velocity and smooth finite inflation;
% their deployment changes force continuously and never resets velocity.
% Earth rotation and canopy swing are omitted. Centers are constrained to +Z.
% Reference: RocketPy generalized equations and their fixed dry-CM state.
% https://docs.rocketpy.org/en/latest/technical/equations_of_motion_v1.html
% Weather is an explicit
% configuration and gust signal, held during each major integration step.
% Optional absolute body-Cd override is forwarded to the aerodynamic model.
% cdInfo = [nominal;command;enable;effective;bodyScale]; dbg stays 18 elements.
if nargin<5, Weather=[]; end
if nargin<6, gustNED=rocketWeatherGust(t,Weather); end
if nargin<7, cdCommand=0; end
if nargin<8, cdOverrideEnabled=false; end
R=rocketPlantRotation(x(7:10));
w=x(11:13);
hAGL=-x(3);
[windN,rho,pressure,soundSpeed]=rocketWeatherConditions(Rocket.AltitudeMSL+hAGL,Rocket,Weather,gustNED);
[thrust,mass,I,massRate,~,MassProperties]=rocketPlantProperties(t,Rocket);
vAirB=R.'*(x(4:6)-windN);
[fAero,mAero,aero]=rocketPlantAerodynamics(x,t,Rocket,Weather,gustNED, ...
    cdCommand,cdOverrideEnabled,MassProperties);
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
% [nominal Cd; absolute command; enable; effective Cd; active body scale].
% The recovery fade and AeroEnabled switch also scale the axial body drag.
cdInfo=[aero.cdNominal;aero.cdCommand;double(aero.cdOverrideEnabled); ...
    aero.cd;bodyAeroFraction*double(Rocket.AeroEnabled>=0.5)];
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
% All load moments above use FIXED body stations. The coupled formulation
% subtracts the moving CM's force moment without moving those stations or
% using a CM-relative lever arm in their local wind/rotational velocities.
c=MassProperties.CenterOfMass;
cDot=MassProperties.CenterOfMassRate;
cDDot=MassProperties.CenterOfMassAcceleration;
weightB=R.'*[0;0;mass*g];
nozzle=Rocket.ThrustOffset;
T03=2*massRate*(nozzle-c)-2*mass*cDot;
T20=-cross(w,cross(w,mass*c))+cross(w,T03)+forceB+weightB- ...
    mass*cDDot-2*massRate*cDot+MassProperties.MassAcceleration*(nozzle-c);
T21=-cross(w,MassProperties.ReferenceInertia.*w)+ ...
    (massRate*Rocket.NozzleGyration-MassProperties.ReferenceInertiaRate).*w+ ...
    cross(c,weightB)+momentB;
alphaB=(T21-cross(c,T20))./I;
aN=R*(T20/mass+cross(c,alphaB));
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
