function [f_B,B_B,dbg,cdInfo] = rocketPlantForces(x,t,g,B_NED,Rocket,Weather,gustNED,cdCommand,cdOverrideEnabled)
%ROCKETPLANTFORCES Physical IMU specific force and body magnetic field.
% Includes rail/pad/ground support and acceleration at a rigid-body IMU
% offset. IMUOffset is a FIXED station from body reference O in body axes [m].
% Moving-CM acceleration is already included in the reference acceleration
% returned by rocketPlantLoads; the physical IMU station does not move.
% dbg is the fixed 18-vector documented in rocketPlantLoads.
% Optional Cd command/enable use the same body drag as the state integrator.
% cdInfo is the separate five-element control diagnostic from rocketPlantLoads.
if nargin<6, Weather=[]; end
if nargin<7, gustNED=rocketWeatherGust(t,Weather); end
if nargin<8, cdCommand=0; end
if nargin<9, cdOverrideEnabled=false; end
[aN,alphaB,dbg,cdInfo]=rocketPlantLoads(x,t,g,Rocket,Weather,gustNED,cdCommand,cdOverrideEnabled);
R=rocketPlantRotation(x(7:10));
w=x(11:13);
r=Rocket.IMUOffset;
specificAccelerationNED=aN-[0;0;g];
if isfield(Rocket,'IncludeCoriolis') && Rocket.IncludeCoriolis==1 && ...
        x(14)>1.5 && x(14)<4.5
    % Remove the coordinate acceleration included in free-flight translation.
    % Native rail/pad/ground branches do not include that Coriolis term.
    earthRate=2*pi/86164.1;
    earthOmegaNED=earthRate*[cos(Rocket.Latitude);0;-sin(Rocket.Latitude)];
    specificAccelerationNED=specificAccelerationNED+2*cross(earthOmegaNED,x(4:6));
end
f_B=R.'*specificAccelerationNED+cross(alphaB,r)+cross(w,cross(w,r));
B_B=R.'*B_NED;
end
