function report = checkRocketPlant(outputDirectory,Rocket)
%CHECKROCKETPLANT Independent numerical checks of the original rocket plant.
% report = checkRocketPlant() loads the demonstration rocket settings.
% report = checkRocketPlant(outputDirectory,Rocket) optionally writes a JSON
% report and tests a supplied numeric configuration. No Simulink model is
% opened, changed or simulated. All modified configurations are local copies.
% Assertions use closed-form mechanics or physical invariants, not a CSV or
% another rocket simulator as a reference. Full-flight checks are numerical
% validation of this approximation, not validation against a real rocket.

if nargin<1
    outputDirectory = '';
end
navigationDirectory = fileparts(fileparts(mfilename('fullpath')));
previousPath = path;
pathCleanup = onCleanup(@() path(previousPath)); %#ok<NASGU>
addpath(fullfile(navigationDirectory,'functions','aux_Rocket'));
if nargin<2 || isempty(Rocket)
    Rocket = loadDemonstrationSettings(navigationDirectory);
end
rocketPlantValidateConfig(Rocket);

names = {'vacuumBallistic','constantThrust','propellantConservation', ...
    'bodyFixedQuaternion','imuAndMagneticFrames','railOrientation', ...
    'finiteDeployment','fullFlightAndConvergence'};
checks = {@() checkBallistic(Rocket), @() checkConstantThrust(Rocket), ...
    @() checkPropellant(Rocket), @() checkQuaternion(Rocket), ...
    @() checkImuAndMagnetic(Rocket), @() checkRail(Rocket), ...
    @() checkDeployments(Rocket), @() checkFullFlight(Rocket)};
results = repmat(struct('Name','','Passed',false,'ElapsedSeconds',0, ...
    'Metrics',struct(),'Message',''),numel(checks),1);
for k=1:numel(checks)
    results(k).Name = names{k};
    started = tic;
    try
        results(k).Metrics = checks{k}();
        results(k).Passed = true;
        fprintf('PASS %s\n',names{k});
    catch exception
        results(k).Message = exception.message;
        fprintf('FAIL %s: %s\n',names{k},exception.message);
    end
    results(k).ElapsedSeconds = toc(started);
end
report = struct('Passed',all([results.Passed]), ...
    'CreatedUTC',char(datetime('now','TimeZone','UTC')), ...
    'RocketSettings',Rocket,'Checks',results);
if ~isempty(outputDirectory)
    if ~isfolder(outputDirectory)
        mkdir(outputDirectory);
    end
    writeReport(fullfile(outputDirectory,'rocket-plant-checks.json'),report);
end
failedNames = {results(~[results.Passed]).Name};
assert(report.Passed,'RocketPlantCheck:Failed', ...
    'Rocket plant checks failed: %s.',strjoin(failedNames,', '));
end

function Rocket = loadDemonstrationSettings(navigationDirectory)
run(fullfile(navigationDirectory,'settings','rocketPlantSettings.m'));
end

function R = inertRocket(R)
% Keep a valid positive-impulse motor, but place ignition beyond these tests.
R.IgnitionTime = 1e6;
R.PropellantMass = 0;
R.PropellantInertia = zeros(3,1);
R.AeroCdPowered = zeros(size(R.AeroCdPowered));
R.AeroCdCoast = zeros(size(R.AeroCdCoast));
R.AeroEnabled = 0;
R.AngularDamping = zeros(3,1);
R.WindNED = zeros(size(R.WindNED));
R.DrogueCdS = 0;
R.MainCdS = 0;
R.ThrustOffset = zeros(3,1);
R.IMUOffset = zeros(3,1);
R.Ts = 0.02;
R.MaxStep = 0.002;
end

function x = freeState(R)
x = rocketPlantInitialize(R);
x(1:3) = [0;0;-1000];
x(7:10) = [0;0;0;1];
x(14) = 2;
end

function x = advance(x,t,duration,g,R)
steps = round(duration/R.Ts);
assert(abs(steps*R.Ts-duration)<1e-12, ...
    'RocketPlantCheck:TestGrid','Test duration must match the output grid.');
for k=1:steps
    x = rocketPlantStep(x,t+(k-1)*R.Ts,g,R);
end
end

function metrics = checkBallistic(R)
R = inertRocket(R);
g = 9.80665;
initial = freeState(R);
initial(1:3) = [30;-20;-1000];
initial(4:6) = [13;-7;-11];
duration = 1.2;
actual = advance(initial,0,duration,g,R);
gravity = [0;0;g];
expectedPosition = initial(1:3)+duration*initial(4:6)+0.5*duration^2*gravity;
expectedVelocity = initial(4:6)+duration*gravity;
positionError = norm(actual(1:3)-expectedPosition);
velocityError = norm(actual(4:6)-expectedVelocity);
assert(positionError<2e-9 && velocityError<2e-10, ...
    'RocketPlantCheck:Ballistic','Vacuum trajectory disagrees with constant gravity.');
assert(norm(actual(7:13)-initial(7:13))<1e-12, ...
    'RocketPlantCheck:BallisticAttitude','Torque-free nonrotating state changed attitude.');
metrics = struct('PositionErrorM',positionError,'VelocityErrorMps',velocityError);
end

function metrics = checkConstantThrust(R)
R = inertRocket(R);
R.IgnitionTime = 0;
R.ThrustTime = [0;0.1;5;5.1];
force = 2.3*R.DryMass*9.80665;
R.ThrustForce = [0;force;force;0];
R.ThrustCumulativeImpulse = cumtrapz(R.ThrustTime,R.ThrustForce);
R.ThrustCumulativeImpulse = R.ThrustCumulativeImpulse/R.ThrustCumulativeImpulse(end);
rocketPlantValidateConfig(R);
g = 9.80665;
initial = freeState(R);
initial(7:10) = [1;0;0;0]; % Body +z is vertical Up; a 180-degree x rotation.
initial(4:6) = [3;-2;-4];
start = 0.5;
duration = 0.4;
actual = advance(initial,start,duration,g,R);
acceleration = [0;0;g-force/R.DryMass];
expectedPosition = initial(1:3)+duration*initial(4:6)+0.5*duration^2*acceleration;
expectedVelocity = initial(4:6)+duration*acceleration;
positionError = norm(actual(1:3)-expectedPosition);
velocityError = norm(actual(4:6)-expectedVelocity);
assert(positionError<2e-9 && velocityError<2e-10, ...
    'RocketPlantCheck:ConstantThrust','Constant-mass vertical thrust has incorrect direction or magnitude.');
[specificForce,~,debug] = rocketPlantForces(initial,start,g,zeros(3,1),R);
assert(norm(specificForce-[0;0;force/R.DryMass])<1e-10 && ...
    abs(debug(2)-R.DryMass)<1e-12, ...
    'RocketPlantCheck:ThrustIMU','Thrust specific force or constant mass is incorrect.');
metrics = struct('PositionErrorM',positionError,'VelocityErrorMps',velocityError, ...
    'VerticalAccelerationDownMps2',acceleration(3));
end

function metrics = checkPropellant(R)
totalImpulse = trapz(R.ThrustTime,R.ThrustForce);
sampleTimes = sort(unique([R.ThrustTime; ...
    0.5*(R.ThrustTime(1:end-1)+R.ThrustTime(2:end))]));
actualMass = zeros(size(sampleTimes));
maxMassError = 0;
maxFlowError = 0;
for k=1:numel(sampleTimes)
    tau = sampleTimes(k);
    % Include every thrust knot before the query. Trapezoidal quadrature is
    % exact for this piecewise-linear input, including an interior query.
    integrationTimes = [R.ThrustTime(R.ThrustTime<tau);tau];
    usedImpulse = 0;
    if numel(integrationTimes)>1
        usedImpulse = trapz(integrationTimes, ...
            interp1(R.ThrustTime,R.ThrustForce,integrationTimes,'linear'));
    end
    expectedMass = R.DryMass+R.PropellantMass*(1-usedImpulse/totalImpulse);
    expectedFlow = -R.PropellantMass* ...
        interp1(R.ThrustTime,R.ThrustForce,tau,'linear')/totalImpulse;
    [~,actualMass(k),~,actualFlow] = ...
        rocketPlantProperties(R.IgnitionTime+tau,R);
    maxMassError = max(maxMassError,abs(actualMass(k)-expectedMass));
    maxFlowError = max(maxFlowError,abs(actualFlow-expectedFlow));
end
integrationTimes = linspace(0,R.ThrustTime(end),1001).';
integrationTimes = sort(unique([integrationTimes;R.ThrustTime]));
flow = zeros(size(integrationTimes));
for k=1:numel(integrationTimes)
    [~,~,~,flow(k)] = rocketPlantProperties(R.IgnitionTime+integrationTimes(k),R);
end
consumedMass = -trapz(integrationTimes,flow);
[thrustBefore,wetMass,wetInertia] = rocketPlantProperties(R.IgnitionTime-0.1,R);
[thrustAfter,dryMass,dryInertia] = ...
    rocketPlantProperties(R.IgnitionTime+R.ThrustTime(end)+0.1,R);
assert(maxMassError<1e-10 && maxFlowError<1e-10, ...
    'RocketPlantCheck:MotorMass','Motor mass or mass flow disagrees with integrated thrust.');
assert(abs(consumedMass-R.PropellantMass)<1e-9 && all(diff(actualMass)<=1e-10), ...
    'RocketPlantCheck:PropellantConservation','Burn does not conserve propellant mass.');
assert(abs(wetMass-R.DryMass-R.PropellantMass)<1e-12 && ...
    abs(dryMass-R.DryMass)<1e-12 && thrustBefore==0 && thrustAfter==0 && ...
    norm(wetInertia-R.DryInertia-R.PropellantInertia)<1e-12 && ...
    norm(dryInertia-R.DryInertia)<1e-12, ...
    'RocketPlantCheck:MotorEndpoints','Preburn or postburn motor properties are incorrect.');
metrics = struct('TotalImpulseNs',totalImpulse,'ConsumedPropellantKg',consumedMass, ...
    'MaximumMassErrorKg',maxMassError,'MaximumMassFlowErrorKgps',maxFlowError);
end

function metrics = checkQuaternion(R)
R = inertRocket(R);
R.DryInertia = [3;3;3]; % Spherical inertia makes the torque-free body rate constant.
initial = freeState(R);
initial(7:10) = [0;0;sqrt(0.5);sqrt(0.5)]; % Initial yaw of +90 degrees.
omega = 1.1;
initial(11:13) = [0;omega;0];
duration = 0.6;
actual = advance(initial,0,duration,0,R);
angle = omega*duration;
s = sin(angle/2)*sqrt(0.5);
c = cos(angle/2)*sqrt(0.5);
expectedQuaternion = [-s;s;c;c];
quaternionError = min(norm(actual(7:10)-expectedQuaternion), ...
    norm(actual(7:10)+expectedQuaternion));
rotation = rocketPlantRotation(actual(7:10));
expectedNose = [0;sin(angle);cos(angle)];
assert(quaternionError<2e-10 && norm(rotation(:,3)-expectedNose)<2e-10, ...
    'RocketPlantCheck:BodyRateQuaternion','Body-fixed rate is not a right Hamilton rotation into NED.');
assert(norm(actual(11:13)-initial(11:13))<1e-11, ...
    'RocketPlantCheck:TorqueFreeRate','Spherical torque-free body rate changed.');
metrics = struct('QuaternionError',quaternionError, ...
    'BodyZDirectionError',norm(rotation(:,3)-expectedNose));
end

function metrics = checkImuAndMagnetic(R)
R = inertRocket(R);
g = 9.80665;
field = [21000;4000;41000];
falling = freeState(R);
falling(7:10) = [0;0;sqrt(0.5);sqrt(0.5)];
falling(4:6) = [6;-2;5];
[fallingForce,bodyField] = rocketPlantForces(falling,0,g,field,R);
expectedField = [field(2);-field(1);field(3)];
assert(norm(fallingForce)<1e-11, ...
    'RocketPlantCheck:FreefallIMU','Freefall accelerometer should measure zero specific force.');
assert(norm(bodyField-expectedField)<1e-8 && abs(norm(bodyField)-norm(field))<1e-8, ...
    'RocketPlantCheck:MagneticRotation','Magnetic body transformation has incorrect sign or norm.');
stationary = falling;
stationary(4:6) = zeros(3,1);
stationary(14) = 0;
[staticForce,~,debug] = rocketPlantForces(stationary,0,g,field,R);
assert(norm(staticForce-[0;0;-g])<1e-11 && norm(debug(15:17))<1e-12, ...
    'RocketPlantCheck:StaticIMU','Supported stationary IMU should measure minus gravity.');
falling(1:3) = [7;-9;-123];
falling(4:6) = [2;3;4];
[q,w,pNEU,vNEU,pressure] = rocketPlantKinematics(falling,R);
assert(norm(pNEU-[7;-9;R.AltitudeMSL+123])<1e-12 && ...
    norm(vNEU-[2;3;-4])<1e-12 && norm(q-falling(7:10))<1e-12 && ...
    norm(w-falling(11:13))<1e-12 && isfinite(pressure) && pressure>0, ...
    'RocketPlantCheck:EnvironmentInterface','NED to absolute-height NEU interface is incorrect.');
% Offset centripetal force is physical even when the centre of mass falls freely.
R.IMUOffset = [0.2;0;0];
R.DryInertia = [3;3;3];
falling(11:13) = [0;0;2];
[offsetForce,~] = rocketPlantForces(falling,0,g,field,R);
assert(norm(offsetForce-[-0.8;0;0])<1e-11, ...
    'RocketPlantCheck:IMULeverArm','Rigid IMU centripetal acceleration is incorrect.');
metrics = struct('FreefallSpecificForceNormMps2',norm(fallingForce), ...
    'MagneticNormError',abs(norm(bodyField)-norm(field)), ...
    'StaticSpecificForceNormMps2',norm(staticForce));
end

function metrics = checkRail(R)
R.LaunchInclination = 85*pi/180;
R.LaunchHeading = 30*pi/180;
R.LaunchRoll = 0;
initial = rocketPlantInitialize(R);
rotation = rocketPlantRotation(initial(7:10));
expectedNose = [cos(85*pi/180)*cos(30*pi/180); ...
    cos(85*pi/180)*sin(30*pi/180);-sin(85*pi/180)];
assert(norm(rotation(:,3)-expectedNose)<1e-12 && ...
    norm(rotation.'*rotation-eye(3),'fro')<1e-12 && abs(det(rotation)-1)<1e-12, ...
    'RocketPlantCheck:RailAttitude','Launch rail body +z direction or rotation handedness is incorrect.');
[specificForce,~,~] = rocketPlantForces(initial,0,9.80665,zeros(3,1),R);
expectedStaticForce = 9.80665*[-cos(85*pi/180);0;sin(85*pi/180)];
assert(norm(specificForce-expectedStaticForce)<1e-10, ...
    'RocketPlantCheck:RailGravity','Rail attitude gives incorrect body gravity components.');
assert(all(initial(1:6)==0) && all(initial(11:14)==0) && all(initial(15:17)==-1), ...
    'RocketPlantCheck:InitialState','Pad state or deployment initialization is incorrect.');
metrics = struct('NoseNED',rotation(:,3),'InclinationDeg',85,'HeadingDeg',30);
end

function metrics = checkDeployments(R)
base = R;
R = inertRocket(R);
R.DrogueCdS = base.DrogueCdS;
R.MainCdS = base.MainCdS;
R.DrogueAttachment = zeros(3,1);
R.MainAttachment = zeros(3,1);
assert(R.DrogueCdS>0 && R.MainCdS>0, ...
    'RocketPlantCheck:RecoveryConfiguration','Two-canopy tests require positive canopy areas.');
state = freeState(R);
state(4:6) = [2;-1;20];
state(15) = 0;
state(16) = 1;
smallStep = R;
smallStep.Ts = 1e-6;
smallStep.MaxStep = smallStep.Ts;
[beforeForce,~,~] = rocketPlantForces(state,1-1e-7,9.80665,zeros(3,1),R);
[atForce,~,startDebug] = rocketPlantForces(state,1,9.80665,zeros(3,1),R);
triggered = rocketPlantStep(state,1,9.80665,smallStep);
drogueContinuityError = norm(triggered(4:6)-state(4:6)-[0;0;9.80665]*smallStep.Ts);
assert(startDebug(9)==0 && norm(atForce-beforeForce)<1e-12 && triggered(14)==3, ...
    'RocketPlantCheck:DrogueOnset','Drogue force is discontinuous at activation.');
assert(drogueContinuityError<1e-9 && ...
    norm(triggered(7:13)-state(7:13))<1e-11, ...
    'RocketPlantCheck:DrogueContinuity','Drogue activation reset velocity or attitude.');
drogueAreas = sampleCanopyAreas(state,1,R.DrogueInflationTime,9,R);
assert(drogueAreas(1)==0 && all(diff(drogueAreas)>=0) && ...
    abs(drogueAreas(end)-R.DrogueCdS)<1e-12, ...
    'RocketPlantCheck:DrogueInflation','Drogue inflation is not finite, monotone and bounded.');

state(14) = 3;
mainTime = 2+R.DrogueInflationTime;
state(17) = mainTime;
[beforeForce,~,~] = rocketPlantForces(state,mainTime-1e-7,9.80665,zeros(3,1),R);
[atForce,~,startDebug] = rocketPlantForces(state,mainTime,9.80665,zeros(3,1),R);
triggered = rocketPlantStep(state,mainTime,9.80665,smallStep);
initialAcceleration = atForce+[0;0;9.80665]; % Identity attitude, zero IMU offset.
mainContinuityError = norm(triggered(4:6)-state(4:6)-initialAcceleration*smallStep.Ts);
assert(startDebug(10)==0 && norm(atForce-beforeForce)<1e-12 && triggered(14)==4, ...
    'RocketPlantCheck:MainOnset','Main force is discontinuous at activation.');
assert(mainContinuityError<1e-8 && ...
    norm(triggered(7:13)-state(7:13))<1e-11, ...
    'RocketPlantCheck:MainContinuity','Main activation reset velocity or attitude.');
mainAreas = sampleCanopyAreas(state,mainTime,R.MainInflationTime,10,R);
assert(mainAreas(1)==0 && all(diff(mainAreas)>=0) && ...
    abs(mainAreas(end)-R.MainCdS)<1e-12, ...
    'RocketPlantCheck:MainInflation','Main inflation is not finite, monotone and bounded.');
[fullForce,~,fullDebug] = rocketPlantForces(state,mainTime+R.MainInflationTime, ...
    9.80665,zeros(3,1),R);
expectedDragAcceleration = -0.5*fullDebug(7)*(R.DrogueCdS+R.MainCdS)* ...
    norm(state(4:6))*state(4:6)/R.DryMass;
assert(norm(fullForce-expectedDragAcceleration)<1e-10, ...
    'RocketPlantCheck:RecoveryDrag','Fully inflated canopies do not produce opposing quadratic drag.');
metrics = struct('DrogueCdSSamples',drogueAreas,'MainCdSSamples',mainAreas, ...
    'DrogueVelocityContinuityErrorMps',drogueContinuityError, ...
    'MainVelocityContinuityErrorMps',mainContinuityError);
end

function areas = sampleCanopyAreas(state,deployTime,inflationTime,index,R)
fractions = [0;0.01;0.25;0.5;0.75;1;1.1];
areas = zeros(size(fractions));
for k=1:numel(fractions)
    [force,field,debug] = rocketPlantForces(state, ...
        deployTime+fractions(k)*inflationTime,9.80665,[2;3;4],R);
    assert(all(isfinite([force;field;debug])), ...
        'RocketPlantCheck:RecoveryFinite','Inflation produced nonfinite loads.');
    areas(k) = debug(index);
end
end

function metrics = checkFullFlight(R)
assert(R.Ts>=0.004 && R.DrogueCdS>0 && R.MainCdS>0, ...
    'RocketPlantCheck:FlightConfiguration','Flight refinement requires Ts>=0.004 and two recovery canopies.');
coarseRocket = R;
coarseRocket.MaxStep = 0.004;
fineRocket = R;
fineRocket.MaxStep = 0.002;
coarse = flightSummary(coarseRocket);
fine = flightSummary(fineRocket);
heightDifference = abs(coarse.ApogeeHeightAGL-fine.ApogeeHeightAGL);
speedDifference = abs(coarse.MaximumGroundSpeed-fine.MaximumGroundSpeed);
apogeeTimeDifference = abs(coarse.ApogeeTime-fine.ApogeeTime);
landingTimeDifference = abs(coarse.LandingTime-fine.LandingTime);
% Event crossings are localized linearly, so the hybrid flight is not
% globally fourth order. These tolerances bound practical output changes.
assert(heightDifference<=0.5 && speedDifference<=0.2 && ...
    apogeeTimeDifference<=0.04 && landingTimeDifference<=0.25, ...
    'RocketPlantCheck:StepConvergence', ...
    'Refining 0.004 to 0.002 s changes apogee %.6g m, peak speed %.6g m/s, apogee time %.6g s or landing %.6g s.', ...
    heightDifference,speedDifference,apogeeTimeDifference,landingTimeDifference);
metrics = struct('Coarse',coarse,'Fine',fine,'ApogeeHeightDifferenceM',heightDifference, ...
    'PeakSpeedDifferenceMps',speedDifference,'ApogeeTimeDifferenceS',apogeeTimeDifference, ...
    'LandingTimeDifferenceS',landingTimeDifference,'HeightToleranceM',0.5, ...
    'SpeedToleranceMps',0.2);
end

function summary = flightSummary(R)
g = 9.80665;
state = rocketPlantInitialize(R);
maximumHeight = 0;
maximumSpeed = 0;
maximumRate = 0;
maximumQuaternionError = 0;
previousPhase = 0;
seenPhases = false(6,1);
seenPhases(1) = true;
maximumTime = R.IgnitionTime+400;
landed = false;
for k=1:ceil(maximumTime/R.Ts)
    time = (k-1)*R.Ts;
    state = rocketPlantStep(state,time,g,R);
    [force,field,debug] = rocketPlantForces(state,time+R.Ts,g,[21000;4000;41000],R);
    assert(all(isfinite([state;force;field;debug])), ...
        'RocketPlantCheck:FiniteFlight','Full flight produced a nonfinite state or load.');
    phase = state(14);
    assert(abs(phase-round(phase))<1e-12 && phase>=previousPhase && phase<=5, ...
        'RocketPlantCheck:FlightPhases','Flight phase moved backwards or became invalid.');
    seenPhases(round(phase)+1) = true;
    previousPhase = phase;
    maximumHeight = max(maximumHeight,-state(3));
    maximumSpeed = max(maximumSpeed,norm(state(4:6)));
    maximumRate = max(maximumRate,norm(state(11:13)));
    maximumQuaternionError = max(maximumQuaternionError,abs(norm(state(7:10))-1));
    if phase==5
        landed = true;
        landingTime = time+R.Ts;
        break
    end
end
assert(landed && all(seenPhases), ...
    'RocketPlantCheck:RecoverySequence','Flight did not visit pad, rail, free, drogue, main and landed phases within 400 s of ignition.');
assert(state(15)>R.IgnitionTime && state(16)>=state(15) && ...
    state(17)>state(16) && landingTime>state(17) && maximumHeight>R.MainAltitudeAGL, ...
    'RocketPlantCheck:RecoveryOrder','Apogee, drogue, main or ground events are not physically ordered.');
assert(maximumQuaternionError<1e-10 && abs(state(3))<1e-12 && ...
    norm(state(4:6))<1e-12 && norm(state(11:13))<1e-12, ...
    'RocketPlantCheck:LandedState','Quaternion norm or supported landed state is incorrect.');
held = rocketPlantStep(state,landingTime,g,R);
assert(norm(held-state)<1e-12, ...
    'RocketPlantCheck:GroundHold','Landed state did not remain supported and stationary.');
summary = struct('ApogeeHeightAGL',maximumHeight,'MaximumGroundSpeed',maximumSpeed, ...
    'MaximumBodyRateRadps',maximumRate,'ApogeeTime',state(15), ...
    'DrogueTime',state(16),'MainTime',state(17),'LandingTime',landingTime, ...
    'MaximumQuaternionNormError',maximumQuaternionError,'Samples',k, ...
    'MaxStep',R.MaxStep);
end

function writeReport(filename,report)
file = fopen(filename,'w');
assert(file>=0,'RocketPlantCheck:ReportFile','Cannot write report: %s.',filename);
fileCleanup = onCleanup(@() fclose(file)); %#ok<NASGU>
fprintf(file,'%s\n',jsonencode(report,PrettyPrint=true));
end
