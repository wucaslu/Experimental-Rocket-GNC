function [accelerationBody,angularAccelerationBody] = ...
    rocketPlantMomentum(omegaBody,forceBody,momentBody,weightBody,Geometry,Rocket)
%ROCKETPLANTMOMENTUM Solve coupled translation and rotation about fixed O.
% INPUTS
%   omegaBody [3x1 rad/s]: body angular velocity.
%   forceBody [3x1 N], momentBody [3x1 N*m]: nongravitational loads about O.
%   weightBody [3x1 N]: gravity force expressed in body coordinates.
%   Geometry: mass/rate [kg,kg/s], physical O-to-CM vector and two rates,
%       CM/reference diagonal inertias and reference-inertia rate (SI).
%   Rocket: physical nozzle station [m], NozzleGyration [3x1 m^2], and
%       MomentumModel: 0=physical O-to-CM derivation; 1=RocketPy 1.13 code.
% OUTPUTS
%   accelerationBody [3x1 m/s^2]: fixed-reference acceleration in body axes.
%   angularAccelerationBody [3x1 rad/s^2]: body angular-rate derivative.
% Mode 1 reproduces native com_to_cdm/nozzle_to_cdm vector directions and
% the native translation-angular coupling together. Physical geometry is
% not changed. This compatibility convention differs from RocketPy's
% published fixed-origin derivation in some offset-CM analytic limits.
% No frame conversion, constraints, Earth rotation or hidden state is used.
% https://docs.rocketpy.org/en/latest/_modules/rocketpy/simulation/flight.html
% https://docs.rocketpy.org/en/latest/technical/equations_of_motion_v1.html
nativeRocketPy=false;
if isfield(Rocket,'MomentumModel'), nativeRocketPy=Rocket.MomentumModel==1; end
center=Geometry.CenterOfMass;
centerRate=Geometry.CenterOfMassRate;
centerAcceleration=Geometry.CenterOfMassAcceleration;
nozzle=Rocket.ThrustOffset;
couplingSign=1;
if nativeRocketPy
    center=-center;
    centerRate=-centerRate;
    centerAcceleration=-centerAcceleration;
    nozzle=-nozzle;
    couplingSign=-1;
end
mass=Geometry.Mass;
massRate=Geometry.MassRate;
T03=2*massRate*(nozzle-center)-2*mass*centerRate;
T20=-cross(omegaBody,cross(omegaBody,mass*center))+cross(omegaBody,T03) + ...
    forceBody+weightBody-mass*centerAcceleration-2*massRate*centerRate + ...
    Geometry.MassAcceleration*(nozzle-center);
T21=-cross(omegaBody,Geometry.ReferenceInertia.*omegaBody) + ...
    (massRate*Rocket.NozzleGyration-Geometry.ReferenceInertiaRate).*omegaBody + ...
    cross(center,weightBody)+momentBody;
angularAccelerationBody=(T21-cross(center,T20))./Geometry.InertiaCM;
accelerationBody=T20/mass+couplingSign*cross(center,angularAccelerationBody);
end
