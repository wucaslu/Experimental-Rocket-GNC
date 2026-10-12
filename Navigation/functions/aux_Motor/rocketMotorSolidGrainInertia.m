function [inertia,inertiaRate,Grain] = rocketMotorSolidGrainInertia(mass,massRate,Properties)
%ROCKETMOTORSOLIDGRAININERTIA Reconstruct symmetric BATES grain inertia.
%   mass and massRate are actual remaining solid propellant [kg] and its
%   derivative [kg/s]. Properties contains the prepared grain count, density
%   [kg/m^3], outer/initial inner radii, initial height and separation [m].
%   inertia and inertiaRate are intrinsic diagonal stack values about the
%   fixed propellant centroid [3x1 kg*m^2] and [3x1 kg*m^2/s]. Grain contains
%   Web, InnerRadius and Height [m]. Outer surfaces are inhibited; the bore
%   and both end faces regress equally. Grain centers retain their initial
%   pitch. Geometry follows the supplied mass history, not a combustion law.
%   Bounded inversion and analytic rates require no stored regression state.
%#codegen
assert(isscalar(mass) && isreal(mass) && isfinite(mass) && mass>=0 && ...
    mass<=Properties.FuelInitialMass*(1+8*eps), ...
    'RocketMotor:GrainRuntimeMass','Remaining grain mass is outside its initial bounds.');
assert(isscalar(massRate) && isreal(massRate) && isfinite(massRate) && massRate<=0, ...
    'RocketMotor:GrainRuntimeRate','Grain mass rate must be finite and nonpositive.');
outerRadius = Properties.GrainOuterRadius;
innerInitial = Properties.GrainInitialInnerRadius;
heightInitial = Properties.GrainInitialHeight;
maximumWeb = min(outerRadius-innerInitial,heightInitial/2);
if mass<=0
    % A spent grain carries neither inertia nor inertia rate. The motor
    % supplies zero mass rate after the static-fire curve ends.
    web = maximumWeb;
elseif mass>=Properties.FuelInitialMass
    web = 0;
else
    lower = 0;
    upper = maximumWeb;
    volumeMassFactor = Properties.GrainCount*Properties.GrainDensity*pi;
    for iteration=1:52
        middle = (lower+upper)/2;
        radius = innerInitial+middle;
        height = max(0,heightInitial-2*middle);
        candidateMass = volumeMassFactor*(outerRadius-radius)*(outerRadius+radius)*height;
        if candidateMass>mass
            lower = middle;
        else
            upper = middle;
        end
    end
    web = (lower+upper)/2;
end
radius = min(outerRadius,innerInitial+web);
height = max(0,heightInitial-2*web);
Grain = struct('Web',web,'InnerRadius',radius,'Height',height);
inertia = zeros(3,1);
inertiaRate = zeros(3,1);
if mass>0
    fixedPitch = heightInitial+Properties.GrainSeparation;
    centerVariance = fixedPitch^2*(Properties.GrainCount^2-1)/12;
    transversePerMass = (outerRadius^2+radius^2)/4+height^2/12+centerVariance;
    axialPerMass = (outerRadius^2+radius^2)/2;
    inertia = mass*[transversePerMass;transversePerMass;axialPerMass];
    % m/(dm/dweb) cancels the density factor and remains finite even near
    % simultaneous radial/end burnout; avoid dividing by a tiny web rate.
    annulus = (outerRadius-radius)*(outerRadius+radius);
    denominator = radius*height+annulus;
    massOverWebDerivative = 0;
    if denominator>0
        massOverWebDerivative = -annulus*height/(2*denominator);
    end
    transverseDerivative = transversePerMass + ...
        massOverWebDerivative*(radius/2-height/3);
    axialDerivative = axialPerMass+massOverWebDerivative*radius;
    inertiaRate = massRate*[transverseDerivative;transverseDerivative;axialDerivative];
end
end
