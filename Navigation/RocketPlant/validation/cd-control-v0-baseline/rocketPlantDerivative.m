function dx = rocketPlantDerivative(x,t,g,Rocket,Weather,gustNED)
%ROCKETPLANTDERIVATIVE Continuous states; hybrid event states have zero rate.
% Local NED translation, body rates, active Hamilton scalar-last attitude.
%#codegen
if nargin<5, Weather=[]; end
if nargin<6, gustNED=rocketWeatherGust(t,Weather); end
dx=zeros(17,1);
if x(14)<0.5 || x(14)>4.5
    return
end
[aN,alphaB]=rocketPlantLoads(x,t,g,Rocket,Weather,gustNED);
dx(1:3)=x(4:6);
dx(4:6)=aN;
q=x(7:10)/max(norm(x(7:10)),eps);
w=x(11:13);
dx(7:9)=0.5*(q(4)*w+cross(q(1:3),w));
dx(10)=-0.5*dot(q(1:3),w);
dx(11:13)=alphaB;
if x(14)<1.5
    initial=rocketPlantInitialize(Rocket);
    launchR=rocketPlantRotation(initial(7:10));
    rail=launchR(:,3);
    dx(1:3)=rail*dot(x(4:6),rail);
    dx(7:13)=zeros(7,1);
end
end
