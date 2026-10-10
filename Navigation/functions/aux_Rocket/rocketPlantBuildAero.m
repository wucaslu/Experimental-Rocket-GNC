function Rocket = rocketPlantBuildAero(Rocket)
%ROCKETPLANTBUILDAERO Derive nose, fin-set and tail geometry coefficients.
% Body +Z is noseward; input reference positions are relative to the common
% plant CM. Surface-local CP distances are positive aft, hence subtraction
% from the nose tip, fin root leading edge or tail top body coordinate.
% The flat-plate fin coefficient at runtime is numerator /
% (2 + sqrt((beta(Mach)*planform)^2 + 4)); the fin-set multiplier is separate.
% Barrowman/Diederich geometry and roll corrections follow these references:
% https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/fins/_base_fin.html
% https://raw.githubusercontent.com/RocketPy-Team/RocketPy/master/rocketpy/rocket/aero_surface/fins/_geometry.py
% https://docs.rocketpy.org/en/latest/technical/aerodynamics/roll_equations.html

R = Rocket.BodyRadius;
cr = Rocket.FinRootChord;
ct = Rocket.FinTipChord;
s = Rocket.FinSpan;
sweep = Rocket.FinSweepLength;
n = Rocket.FinNumber;
cant = Rocket.FinCant;
assert(R>0 && cr>0 && ct>=0 && s>0 && n>=3 && n==fix(n), ...
    'RocketPlant:InvalidFinGeometry', ...
    'Radius, root chord and exposed span must be positive; use at least three fins.');

area = pi*R^2;
diameter = 2*R;
Af = (cr+ct)*s/2;
AR = 2*s^2/Af;
gamma = atan((sweep+(ct-cr)/2)/s);
Yma = s*(cr+2*ct)/(3*(cr+ct));
tau = (s+R)/R;
lambda = ct/cr;
kLift = 1+1/tau;

% Integral of chord(y)*(R+y)^2 over the exposed fin span [m^4].
G = ((cr+3*ct)*s^3 + 4*(cr+2*ct)*R*s^2 + ...
    6*(cr+ct)*s*R^2)/12;

% Regroup the standard fin/body roll-forcing expression. atan and log1p
% evaluate the same angle/logarithm without subtracting nearly equal radii.
deltaTau = s/R;
angle = atan(deltaTau*(tau+1)/(2*tau));
B = (tau^2+1)^2/(tau^2*deltaTau^2);
C = (tau+1)/(tau*deltaTau);
kForce = ((pi^2/4)*((tau+1)/tau)^2 + ...
    B*(pi*angle+angle^2) - C*(2*pi+4*angle) + ...
    8*log1p(deltaTau^2/(2*tau))/deltaTau^2)/pi^2;

% The denominator is the exact factored form of the trapezoidal damping
% correction; factoring avoids cancellation near tau=1.
dampNumerator = (tau-lambda)/tau - ...
    (1-lambda)*log1p(deltaTau)/deltaTau;
dampDenominator = deltaTau*((1+lambda)/2 + deltaTau*(1+2*lambda)/6);
kDamp = 1+dampNumerator/dampDenominator;
assert(isfinite(kForce) && kForce>0 && isfinite(kDamp) && kDamp>0, ...
    'RocketPlant:InvalidFinInterference', ...
    'Fin/body interference is nonfinite or nonpositive; check exposed span and geometry.');

if n>=5 && n<=8
    finCorrection = [2.37 2.74 2.99 3.24];
    nFactor = finCorrection(n-4);
else
    nFactor = n/2;
end

noseLocalCP = Rocket.NoseCPFraction*Rocket.NoseLength;
finLocalCP = sweep*(cr+2*ct)/(3*(cr+ct)) + ...
    (cr+ct-cr*ct/(cr+ct))/6;
tailRadiusSum = Rocket.TailTopRadius+Rocket.TailBottomRadius;
assert(tailRadiusSum>0, 'RocketPlant:InvalidTailGeometry', ...
    'At least one tail radius must be positive.');
% Continuous equal-radius limit is L/2; a zero bottom radius is also valid.
tailLocalCP = Rocket.TailLength*(1+Rocket.TailBottomRadius/tailRadiusSum)/3;

Rocket.AeroSurfaceCP = [0 0 Rocket.NoseTipZ-noseLocalCP; ...
    0 0 Rocket.FinLeadingEdgeZ-finLocalCP; ...
    0 0 Rocket.TailTopZ-tailLocalCP];
Rocket.AeroSurfaceArea = area*ones(3,1);
Rocket.AeroSurfaceDiameter = diameter*ones(3,1);
Rocket.AeroSurfaceCNa = [2*(Rocket.NoseBaseRadius/R)^2; 0; ...
    2*((Rocket.TailBottomRadius/R)^2-(Rocket.TailTopRadius/R)^2)];
Rocket.AeroFinPlanform = [0; AR/cos(gamma); 0];
Rocket.AeroFinLiftNumerator = [0; 2*pi*AR*Af/area; 0];
Rocket.AeroFinLiftMultiplier = [0; nFactor*kLift; 0];
Rocket.AeroRollForceScale = [0; kForce*n*(Yma+R)/diameter; 0];
Rocket.AeroRollDampScale = [0; 2*kDamp*n*cos(cant)*G/(area*diameter^2); 0];
Rocket.AeroSurfaceCant = [0; cant; 0];
% Af [m^2], AR [-], gamma [rad], Yma [m], G [m^4], kLift, kForce, kDamp.
Rocket.AeroFinGeometry = [Af; AR; gamma; Yma; G; kLift; kForce; kDamp];
end
