function [f_B,B_B,dbg] = rocketPlantForces(x,t,g,B_NED,Rocket,Weather,gustNED)
%ROCKETPLANTFORCES Physical IMU specific force and body magnetic field.
% Includes rail/pad/ground support and acceleration at a rigid-body IMU
% offset. IMUOffset is measured from the lumped CM in body axes [m].
% dbg is the fixed 18-vector documented in rocketPlantLoads.
    if nargin<6, Weather=[]; end
    if nargin<7, gustNED=rocketWeatherGust(t,Weather); end
    [aN,alphaB,dbg]=rocketPlantLoads(x,t,g,Rocket,Weather,gustNED);
    R=rocketPlantRotation(x(7:10));
    w=x(11:13);
    r=Rocket.IMUOffset;
    f_B=R.'*(aN-[0;0;g])+cross(alphaB,r)+cross(w,cross(w,r));
    B_B=R.'*B_NED;
end
