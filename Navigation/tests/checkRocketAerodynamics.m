function report = checkRocketAerodynamics(outputDirectory)
%CHECKROCKETAERODYNAMICS Component loads and RocketPy geometry regressions.
% Closed-form force/moment checks and independent source golden coefficients
% exercise the new aerodynamics without modifying or running Simulink.
% Geometry golden values were calculated from the primary RocketPy formulas:
% https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/fins/_base_fin.html
% https://raw.githubusercontent.com/RocketPy-Team/RocketPy/master/rocketpy/rocket/aero_surface/fins/_geometry.py
% https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/tail.html
% This validates the implementation, not the accuracy of assumed rocket data.
if nargin<1
    outputDirectory='';
end
navigationDirectory=fileparts(fileparts(mfilename('fullpath')));
previousPath=path;
pathCleanup=onCleanup(@() path(previousPath)); %#ok<NASGU>
addpath(fullfile(navigationDirectory,'functions','aux_Rocket'));
Rocket=loadSettings(navigationDirectory);
names={'geometryGolden','axialDragAndMotorMode','componentForceAndMomentSum', ...
    'restoringMomentAndPitchDamping','finCantAndRollDamping', ...
    'transonicFiniteAndLiftGolden','surfaceWindHeight', ...
    'zeroAirspeedAndDisable','explicitStateSource'};
checks={@() checkGeometry(Rocket),@() checkAxialDrag(Rocket), ...
    @() checkComponentSum(Rocket),@() checkPitchDamping(Rocket), ...
    @() checkRoll(Rocket),@() checkTransonic(Rocket), ...
    @() checkWindHeight(Rocket),@() checkZeroAndDisable(Rocket), ...
    @() checkSource(navigationDirectory)};
results=repmat(struct('Name','','Passed',false,'ElapsedSeconds',0, ...
    'Metrics',struct(),'Message',''),numel(checks),1);
for k=1:numel(checks)
    results(k).Name=names{k};
    started=tic;
    try
        results(k).Metrics=checks{k}();
        results(k).Passed=true;
        fprintf('PASS %s\n',names{k});
    catch exception
        results(k).Message=exception.message;
        fprintf('FAIL %s: %s\n',names{k},exception.message);
    end
    results(k).ElapsedSeconds=toc(started);
end
report=struct('Passed',all([results.Passed]), ...
    'CreatedUTC',char(datetime('now','TimeZone','UTC')), ...
    'Checks',results);
if ~isempty(outputDirectory)
    if ~isfolder(outputDirectory)
        mkdir(outputDirectory);
    end
    filename=fullfile(outputDirectory,'rocket-aerodynamics-checks.json');
    file=fopen(filename,'w');
    assert(file>=0,'RocketAeroCheck:ReportFile','Cannot write %s.',filename);
    fileCleanup=onCleanup(@() fclose(file)); %#ok<NASGU>
    fprintf(file,'%s\n',jsonencode(report,PrettyPrint=true));
end
assert(report.Passed,'RocketAeroCheck:Failed','Aerodynamic checks failed: %s.', ...
    strjoin({results(~[results.Passed]).Name},', '));
end

function Rocket=loadSettings(navigationDirectory)
run(fullfile(navigationDirectory,'settings','rocketPlantSettings.m'));
end

function R=fixedGeometry(R)
% This fixed geometry is independent of evolving demonstration settings.
R.BodyRadius=0.05;
R.ReferenceArea=pi*R.BodyRadius^2;
R.NoseLength=0.4;
R.NoseBaseRadius=0.05;
R.NoseTipZ=1.2;
R.NoseCPFraction=2/3;
R.FinNumber=4;
R.FinRootChord=0.2;
R.FinTipChord=0.1;
R.FinSpan=0.1;
R.FinSweepLength=0.04;
R.FinLeadingEdgeZ=-0.6;
R.FinCant=0.02;
R.TailLength=0.1;
R.TailTopRadius=0.05;
R.TailBottomRadius=0.03;
R.TailTopZ=-1.1;
R.WindNED=zeros(size(R.WindNED));
R.AngularDamping=zeros(3,1);
R.AeroEnabled=1;
R=rocketPlantBuildAero(R);
end

function R=maskNormals(R)
R.AeroSurfaceCNa(:)=0;
R.AeroFinLiftMultiplier(:)=0;
end

function R=maskDragAndRoll(R)
R.AeroCdPowered(:)=0;
R.AeroCdCoast(:)=0;
R.AeroRollForceScale(:)=0;
R.AeroRollDampScale(:)=0;
end

function x=freeState(R)
x=rocketPlantInitialize(R);
x(1:3)=[0;0;-1000];
x(4:6)=[0;0;100];
x(7:10)=[0;0;0;1];
x(11:13)=zeros(3,1);
x(14)=2;
end

function metrics=checkGeometry(R)
R=fixedGeometry(R);
% Independent golden geometry, using original (unfactored) RocketPy forms.
expectedCP=[0 0 0.9333333333333333;0 0 -0.6566666666666666; ...
    0 0 -1.1458333333333333];
expectedGeometry=[0.015;1.3333333333333333;-0.09966865249116212; ...
    0.044444444444444446;0.00014583333333333337; ...
    1.3333333333333333;0.9349197014944817;1.1971812686469314];
cpError=norm(R.AeroSurfaceCP-expectedCP,'fro');
geometryError=norm(R.AeroFinGeometry-expectedGeometry);
assert(cpError<1e-12 && geometryError<2e-12, ...
    'RocketAeroCheck:GeometryGolden','Component CP or trapezoidal fin geometry disagrees with primary-source golden values.');
assert(norm(R.AeroSurfaceCNa-[2;0;-1.28])<1e-12 && ...
    abs(R.AeroFinPlanform(2)-1.339983416149452)<1e-12 && ...
    abs(R.AeroFinLiftNumerator(2)-16)<1e-12 && ...
    abs(R.AeroFinLiftMultiplier(2)-2.6666666666666665)<1e-12 && ...
    abs(R.AeroRollForceScale(2)-3.531918872312486)<2e-12 && ...
    abs(R.AeroRollDampScale(2)-17.77992631231421)<2e-11, ...
    'RocketAeroCheck:CoefficientGolden','Nose/tail slopes or fin-body/roll coefficients disagree with source golden values.');
finNumbers=3:9;
corrections=[1.5;2;2.37;2.74;2.99;3.24;4.5];
for k=1:numel(finNumbers)
    caseR=R;
    caseR.FinNumber=finNumbers(k);
    caseR=rocketPlantBuildAero(caseR);
    assert(abs(caseR.AeroFinLiftMultiplier(2)-corrections(k)*4/3)<1e-12, ...
        'RocketAeroCheck:FinNumber','Fin-count correction is incorrect for %d fins.',finNumbers(k));
end
equalTail=R;
equalTail.TailBottomRadius=equalTail.TailTopRadius;
equalTail=rocketPlantBuildAero(equalTail);
assert(equalTail.AeroSurfaceCNa(3)==0 && ...
    abs(equalTail.AeroSurfaceCP(3,3)-(R.TailTopZ-R.TailLength/2))<1e-12 && ...
    all(isfinite(equalTail.AeroSurfaceCP(:))), ...
    'RocketAeroCheck:EqualTail','Equal-radius tail should have zero normal slope and finite midpoint CP.');
pointTail=R;
pointTail.TailBottomRadius=0;
pointTail=rocketPlantBuildAero(pointTail);
assert(pointTail.AeroSurfaceCNa(3)<0 && ...
    abs(pointTail.AeroSurfaceCP(3,3)-(R.TailTopZ-R.TailLength/3))<1e-12, ...
    'RocketAeroCheck:PointTail','Zero-radius boattail limit has wrong sign or CP.');
[~,~,sound]=rocketPlantAtmosphere(R.AltitudeMSL+1000);
slopes=[2;goldenSingleFin(100/sound)*8/3;-1.28];
expectedWeightedCP=expectedCP.'*slopes/sum(slopes);
[~,~,aero]=rocketPlantAerodynamics(freeState(R),0,R);
expectedMargin=-expectedWeightedCP(3)/0.1;
assert(norm(aero.cpWeighted-expectedWeightedCP)<1e-12 && ...
    abs(aero.staticMargin-expectedMargin)<1e-11 && aero.staticMargin>0, ...
    'RocketAeroCheck:StaticMargin','Weighted CP or signed static margin in diameters is incorrect.');
metrics=struct('CPErrorM',cpError,'FinGeometryError',geometryError, ...
    'BoattailCNa',R.AeroSurfaceCNa(3),'GoldenFinCountCorrections',corrections, ...
    'StaticMarginDiameters',aero.staticMargin);
end

function metrics=checkAxialDrag(R)
R=maskNormals(fixedGeometry(R));
R.AeroRollForceScale(:)=0;
R.AeroRollDampScale(:)=0;
R.AeroCdPowered(:)=0.25;
R.AeroCdCoast(:)=0.8;
R.ThrustTime=[0;0.5;1;1.5;2];
R.ThrustForce=[0;200;0;200;0];
impulse=cumtrapz(R.ThrustTime,R.ThrustForce);
R.ThrustCumulativeImpulse=impulse/impulse(end);
x=freeState(R);
x(4:6)=[2;3;100];
[rho,~,~]=rocketPlantAtmosphere(R.AltitudeMSL+1000);
q=0.5*rho*dot(x(4:6),x(4:6));
times=R.IgnitionTime+[-1;0.25;1;2-1e-6;2;3];
expectedCd=[0.8;0.25;0.25;0.25;0.8;0.8];
maximumError=0;
for k=1:numel(times)
    [force,moment,aero]=rocketPlantAerodynamics(x,times(k),R);
    expected=[0;0;-q*R.ReferenceArea*expectedCd(k)];
    maximumError=max(maximumError,norm(force-expected));
    assert(norm(force-expected)<1e-10 && norm(moment)<1e-12 && ...
        abs(aero.cd-expectedCd(k))<1e-12, ...
        'RocketAeroCheck:AxialDrag','Axial body drag or burn-window Cd selection is incorrect.');
end
[zeroThrust,~,~]=rocketPlantProperties(R.IgnitionTime+1,R);
assert(zeroThrust==0,'RocketAeroCheck:MotorFixture','Interior zero-thrust fixture is incorrect.');
metrics=struct('MaximumForceErrorN',maximumError,'PoweredCd',0.25, ...
    'CoastCd',0.8,'InteriorZeroThrustTimeS',R.IgnitionTime+1);
end

function metrics=checkComponentSum(R)
R=maskDragAndRoll(maskNormals(fixedGeometry(R)));
% This fixture uses three prescribed constant slopes, including a synthetic
% constant fin slope. Disable its geometry-dependent fin-law selector only
% here; the separate fin/roll/Mach checks retain the derived fin law.
R.AeroFinPlanform(:)=0;
R.AeroSurfaceCP=[0 0 0.4;0 0 -0.5;0 0 -1.2];
R.AeroSurfaceArea=[0.01;0.02;0.03];
R.AeroSurfaceCNa=[2;3;-1];
x=freeState(R);
x(4:6)=[2;1;100];
[rho,~,~]=rocketPlantAtmosphere(R.AltitudeMSL+1000);
speed=norm(x(4:6));
alpha=atan2(sqrt(5),100);
normalDirection=[-2;-1;0]/sqrt(5);
expectedForce=zeros(3,1);
expectedMoment=zeros(3,1);
individualForce=zeros(3,3);
individualMoment=zeros(3,3);
for k=1:3
    component=0.5*rho*speed^2*R.AeroSurfaceArea(k)* ...
        R.AeroSurfaceCNa(k)*alpha*normalDirection;
    torque=cross(R.AeroSurfaceCP(k,:).',component);
    individualForce(k,:)=component.';
    individualMoment(k,:)=torque.';
    expectedForce=expectedForce+component;
    expectedMoment=expectedMoment+torque;
end
[force,moment,aero]=rocketPlantAerodynamics(x,0,R);
forceError=norm(force-expectedForce);
momentError=norm(moment-expectedMoment);
assert(forceError<1e-9 && momentError<1e-9 && ...
    norm(aero.surfaceForce-individualForce,'fro')<1e-9 && ...
    norm(aero.surfaceMoment-individualMoment,'fro')<1e-9, ...
    'RocketAeroCheck:ComponentSum','Component CN/CP forces or summed moments are incorrect.');
assert(individualForce(1,1)<0 && individualForce(3,1)>0, ...
    'RocketAeroCheck:SignedTail','Signed tail normal slope was discarded or folded positive.');
metrics=struct('TotalForceErrorN',forceError,'TotalMomentErrorNm',momentError, ...
    'AngleOfAttackRad',alpha,'ComponentForceN',individualForce);
end

function metrics=checkPitchDamping(R)
R=maskDragAndRoll(maskNormals(fixedGeometry(R)));
R.AeroSurfaceCP(1,:)=[0 0 -0.5];
R.AeroSurfaceCNa(1)=2;
x=freeState(R);
x(4:6)=[2;0;100];
[restoringForce,restoringMoment]=rocketPlantAerodynamics(x,0,R);
assert(restoringForce(1)<0 && restoringMoment(2)>0, ...
    'RocketAeroCheck:RestoringSign','Aft CP should turn the nose toward positive transverse air velocity.');
x(4:6)=[0;0;100];
x(11:13)=[0;0.4;0];
[force,moment]=rocketPlantAerodynamics(x,0,R);
[rho,~,~]=rocketPlantAtmosphere(R.AltitudeMSL+1000);
surfaceVelocity=[-0.2;0;100];
speed=norm(surfaceVelocity);
expectedFx=0.5*rho*speed^2*R.AeroSurfaceArea(1)*2*atan2(0.2,100);
expectedForce=[expectedFx;0;0];
expectedMoment=[0;-0.5*expectedFx;0];
assert(norm(force-expectedForce)<1e-9 && norm(moment-expectedMoment)<1e-9 && ...
    dot(x(11:13),moment)<0, ...
    'RocketAeroCheck:RotationalCP','Local CP rotational velocity does not produce opposing pitch damping.');
R.AngularDamping=[1e6;1e6;1e6];
[changedForce,changedMoment]=rocketPlantAerodynamics(x,0,R);
assert(norm(changedForce-force)<1e-12 && norm(changedMoment-moment)<1e-12, ...
    'RocketAeroCheck:AdHocDamping','Component aerodynamic damping depends on residual lumped AngularDamping.');
x(12)=-0.4;
[~,negativeMoment]=rocketPlantAerodynamics(x,0,R);
assert(negativeMoment(2)>0 && abs(negativeMoment(2)+moment(2))<1e-10, ...
    'RocketAeroCheck:PitchOdd','CP damping does not oppose both signs of pitch rate.');
metrics=struct('RestoringPitchMomentNm',restoringMoment(2), ...
    'DampingPitchMomentNm',moment(2),'PitchMomentOracleNm',expectedMoment(2));
end

function metrics=checkRoll(R)
R=fixedGeometry(R);
R=maskNormals(R);
R.AeroCdPowered(:)=0;
R.AeroCdCoast(:)=0;
x=freeState(R);
[rho,~,sound]=rocketPlantAtmosphere(R.AltitudeMSL+1000);
single=goldenSingleFin(100/sound);
expectedForcing=0.5*rho*100^2*(pi*0.05^2)*0.1* ...
    3.531918872312486*single*0.02;
[~,moment]=rocketPlantAerodynamics(x,0,R);
assert(moment(3)>0 && abs(moment(3)-expectedForcing)<1e-10 && norm(moment(1:2))<1e-12, ...
    'RocketAeroCheck:FinCant','Positive fin cant has incorrect roll forcing sign or magnitude.');
x(13)=2;
[~,spinningMoment]=rocketPlantAerodynamics(x,0,R);
expectedDamping=0.25*rho*100*(pi*0.05^2)*0.1^2* ...
    17.77992631231421*single*2;
assert(abs(spinningMoment(3)-(expectedForcing-expectedDamping))<1e-10 && ...
    spinningMoment(3)<moment(3), ...
    'RocketAeroCheck:RollDamping','Fin roll damping has incorrect magnitude or direction.');
R.FinCant=-0.02;
R=maskNormals(rocketPlantBuildAero(R));
x(13)=0;
[~,negativeCant]=rocketPlantAerodynamics(x,0,R);
assert(abs(negativeCant(3)+moment(3))<1e-10, ...
    'RocketAeroCheck:NegativeCant','Negative cant does not reverse roll forcing.');
R.FinCant=0;
R=maskNormals(rocketPlantBuildAero(R));
x(13)=1;
[~,positiveRate]=rocketPlantAerodynamics(x,0,R);
x(13)=-1;
[~,negativeRate]=rocketPlantAerodynamics(x,0,R);
assert(positiveRate(3)<0 && negativeRate(3)>0 && ...
    abs(positiveRate(3)+negativeRate(3))<1e-10, ...
    'RocketAeroCheck:RollOpposition','Zero-cant roll damping does not oppose both rate signs.');
x(4:6)=0;
x(13)=2;
[zeroForce,zeroMoment]=rocketPlantAerodynamics(x,0,R);
assert(norm(zeroForce)<1e-12 && norm(zeroMoment)<1e-12, ...
    'RocketAeroCheck:ZeroRollAirspeed','Longitudinal roll with zero translation should not create spurious axial-stream fin torque.');
metrics=struct('CantForcingNm',moment(3),'DampingAtTwoRadpsNm',expectedDamping, ...
    'ZeroCantDampingAtOneRadpsNm',positiveRate(3));
end

function metrics=checkTransonic(R)
R=fixedGeometry(R);
R.AeroSurfaceCP(:)=0; % Equal heights remove any atmosphere variation.
R.AeroSurfaceCNa(:)=0;
R=maskDragAndRoll(R);
x=freeState(R);
[rho,~,sound]=rocketPlantAtmosphere(R.AltitudeMSL+1000);
machPoints=[0.3;0.79;0.799999;0.8;0.800001;0.999999;1;1.000001;1.099999;1.100001;2];
actualCNa=zeros(size(machPoints));
maxForceError=0;
alpha=0.02;
for k=1:numel(machPoints)
    speed=machPoints(k)*sound;
    x(4:6)=speed*[sin(alpha);0;cos(alpha)];
    physicalMach=norm(x(4:6))/sound;
    expectedCNa=goldenSingleFin(physicalMach)*8/3;
    [force,moment,aero]=rocketPlantAerodynamics(x,0,R);
    expectedForce=[-0.5*rho*speed^2*(pi*0.05^2)*expectedCNa*alpha;0;0];
    maxForceError=max(maxForceError,norm(force-expectedForce));
    actualCNa(k)=aero.cna(2);
    assert(all(isfinite([force;moment;numericDiagnostics(aero)])) && ...
        abs(aero.mach(2)-physicalMach)<1e-12 && ...
        abs(aero.cna(2)-expectedCNa)<2e-10 && ...
        norm(force-expectedForce)<2e-8 && norm(moment)<1e-12, ...
        'RocketAeroCheck:MachLift','Fin Mach lift disagrees with source oracle or is nonfinite near transonic boundaries.');
end
% RocketPy uses a finite plateau from Mach 0.8 to just below 1.1.
assert(max(actualCNa(4:9))-min(actualCNa(4:9))<1e-10, ...
    'RocketAeroCheck:TransonicPlateau','Compressibility correction has an incorrect transonic plateau.');
metrics=struct('Mach',machPoints,'FinSetCNa',actualCNa,'MaximumForceErrorN',maxForceError);
end

function single=goldenSingleFin(mach)
% Independent fixed-geometry oracle: Diederich denominator and beta law.
if mach<0.8
    beta=sqrt(1-mach^2);
elseif mach<1.1
    beta=0.6;
else
    beta=sqrt(mach^2-1);
end
single=16/(2+sqrt((beta*1.339983416149452)^2+4));
end

function metrics=checkWindHeight(R)
R=maskDragAndRoll(maskNormals(fixedGeometry(R)));
R.AeroSurfaceCNa(1)=2;
R.AeroSurfaceCP(1,:)=[0 0 100];
R.WindAltitude=[0;2000];
R.WindNED=[0 0 0;40 0 0];
x=freeState(R);
x(4:6)=[20;0;100]; % CM is aligned with its own local wind.
surfaceHeight=900; % Identity body-to-NED: +100 m body Z is +100 m Down.
surfaceVelocity=[2;0;100]; % WindN(900 m)=18 m/s, not CM wind=20 m/s.
[rho,~,sound]=rocketPlantAtmosphere(R.AltitudeMSL+1000);
speed=norm(surfaceVelocity);
expectedForce=[-0.5*rho*speed^2*(pi*0.05^2)*2*atan2(2,100);0;0];
expectedMoment=cross([0;0;100],expectedForce);
[force,moment,aero]=rocketPlantAerodynamics(x,0,R);
assert(norm(force-expectedForce)<1e-9 && norm(moment-expectedMoment)<1e-8 && ...
    abs(aero.speed(1)-speed)<1e-12 && abs(aero.mach(1)-speed/sound)<1e-12 && force(1)<0, ...
    'RocketAeroCheck:SurfaceWind','Aerodynamic wind was evaluated at CM height or has incorrect body/altitude sign.');
% Rotate body +Z into North: the same CP is now horizontal, so its height
% equals the CM height and it must see the CM wind rather than hCM-CPz.
x(7:10)=[0;sqrt(0.5);0;sqrt(0.5)];
x(4:6)=[120;0;0];
[horizontalForce,horizontalMoment,horizontalAero]=rocketPlantAerodynamics(x,0,R);
assert(norm(horizontalForce)<1e-10 && norm(horizontalMoment)<1e-8 && ...
    abs(horizontalAero.speed(1)-100)<1e-10, ...
    'RocketAeroCheck:RotatedWindHeight','CP altitude or local wind ignores the body-to-NED rotation.');
% Body +Z Up raises the CP to 1100 m AGL and reverses the transverse stream.
x(7:10)=[1;0;0;0];
x(4:6)=[20;0;-100];
[upForce,upMoment,upAero]=rocketPlantAerodynamics(x,0,R);
assert(norm(upForce+expectedForce)<1e-9 && norm(upMoment+expectedMoment)<1e-8 && ...
    abs(upAero.speed(1)-speed)<1e-12, ...
    'RocketAeroCheck:UpWindHeight','Body +Z Up CP height has the wrong local Down sign.');
metrics=struct('CMHeightAGLM',1000,'SurfaceHeightAGLM',surfaceHeight, ...
    'SurfaceWindNorthMps',18,'SurfaceForceNorthN',force(1), ...
    'HorizontalBodySurfaceAirspeedMps',horizontalAero.speed(1), ...
    'BodyUpSurfaceWindNorthMps',22);
end

function metrics=checkZeroAndDisable(R)
R=fixedGeometry(R);
x=freeState(R);
x(4:6)=0;
[force,moment,aero]=rocketPlantAerodynamics(x,0,R);
assert(norm(force)<1e-12 && norm(moment)<1e-12 && ...
    all(isfinite(numericDiagnostics(aero))), ...
    'RocketAeroCheck:ZeroAirspeed','Zero airspeed produced force, moment or nonfinite diagnostics.');
R.AeroEnabled=0;
x(4:6)=[10;-7;200];
x(11:13)=[2;3;4];
[force,moment,aero]=rocketPlantAerodynamics(x,R.IgnitionTime+1,R);
assert(norm(force)<1e-12 && norm(moment)<1e-12 && ...
    all(isfinite(numericDiagnostics(aero))), ...
    'RocketAeroCheck:Disable','AeroEnabled=0 did not disable body aerodynamic forces and moments.');
metrics=struct('ZeroAirspeedLoads',true,'ExplicitDisableLoads',true);
end

function values=numericDiagnostics(aero)
names=fieldnames(aero);
values=[];
for k=1:numel(names)
    current=aero.(names{k});
    if isnumeric(current)
        values=[values;current(:)]; %#ok<AGROW>
    end
end
end

function metrics=checkSource(navigationDirectory)
files={'rocketPlantAerodynamics.m','rocketPlantBuildAero.m'};
for k=1:numel(files)
    source=fileread(fullfile(navigationDirectory,'functions','aux_Rocket',files{k}));
    assert(isempty(regexp(source,'(?m)^\s*(persistent|global)\b','once')) && ...
        isempty(regexp(source,'\b(rand|randn|rng|evalin|assignin|readmatrix|readtable)\s*\(','once')) && ...
        isempty(regexp(source,'\bcoder\.extrinsic\b','once')), ...
        'RocketAeroCheck:ExplicitState','%s contains hidden state, random input, workspace access or extrinsic calls.',files{k});
end
metrics=struct('FilesChecked',{files},'HiddenStateAbsent',true);
end
