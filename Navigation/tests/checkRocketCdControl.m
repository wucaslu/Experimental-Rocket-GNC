function report = checkRocketCdControl(outputDirectory)
%CHECKROCKETCDCONTROL Physics checks for an absolute body-Cd override.
% Enabled: Cd_effective=cdCommand. Disabled: preserve the nominal Mach/burn
% curve. Checks old-call compatibility, force/frame propagation, isolated
% component loads, analytic quadratic-drag trajectories and canopy CdS.
% No Simulink model is opened or changed. Fixtures are local value copies.
if nargin<1, outputDirectory=''; end
navigationDirectory=fileparts(fileparts(mfilename('fullpath')));
previousPath=path;
pathCleanup=onCleanup(@() path(previousPath));
addpath(fullfile(navigationDirectory,'functions','aux_Rocket'));
[Rocket,Weather]=loadSettings(navigationDirectory);
names={'disabledOverrideCompatibility','absoluteCoefficientForceOracle', ...
    'normalForcesAndMomentsIndependent','frameAndConsumerForwarding', ...
    'analyticCoastAndPoweredTrajectories','majorStepCommandAndContinuity', ...
    'parachuteIndependence','invalidCommandsAndEnables'};
checks={@() checkNeutral(Rocket,Weather),@() checkCoefficient(Rocket,Weather), ...
    @() checkNormalIndependence(Rocket,Weather),@() checkForwarding(Rocket,Weather), ...
    @() checkAnalyticTrajectories(Rocket),@() checkCommandBoundary(Rocket), ...
    @() checkParachutes(Rocket,Weather),@() checkInvalid(Rocket,Weather)};
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
    'CreatedUTC',char(datetime('now','TimeZone','UTC')),'Checks',results);
if ~isempty(outputDirectory)
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    filename=fullfile(outputDirectory,'rocket-cd-control-checks.json');
    file=fopen(filename,'w');
    assert(file>=0,'RocketCdCheck:ReportFile','Cannot write %s.',filename);
    fileCleanup=onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(report,PrettyPrint=true));
end
assert(report.Passed,'RocketCdCheck:Failed','Body Cd control checks failed: %s.', ...
    strjoin({results(~[results.Passed]).Name},', '));
end

function [Rocket,Weather]=loadSettings(navigationDirectory)
Rocket=struct();
Weather=struct();
run(fullfile(navigationDirectory,'settings','rocketPlantSettings.m'));
run(fullfile(navigationDirectory,'settings','weatherSettings.m'));
end

function x=freeState(R)
x=rocketPlantInitialize(R);
x(1:3)=[0;0;-1000];
x(7:10)=[1;0;0;0]; % Body +z Up; forward ascent under body-drag convention.
x(4:6)=[8;-3;-150];
x(11:13)=[0.1;-0.15;0.2];
x(14)=2;
end

function metrics=checkNeutral(R,W)
x=freeState(R);
t=R.IgnitionTime+0.4;
gust=zeros(3,1);
[f0,m0,aero0]=rocketPlantAerodynamics(x,t,R,W,gust);
[f1,m1,aero1]=rocketPlantAerodynamics(x,t,R,W,gust,0,false);
assert(isequal(f0,f1) && isequal(m0,m1) && isequal(aero0,aero1), ...
    'RocketCdCheck:NeutralAero','Explicit disabled override changes nominal aerodynamics.');
[a0,alpha0,d0,info0]=rocketPlantLoads(x,t,9.80665,R,W,gust);
[a1,alpha1,d1,info1]=rocketPlantLoads(x,t,9.80665,R,W,gust,0,false);
dx0=rocketPlantDerivative(x,t,9.80665,R,W,gust);
dx1=rocketPlantDerivative(x,t,9.80665,R,W,gust,0,false);
step0=rocketPlantStep(x,t,9.80665,R,W,gust);
step1=rocketPlantStep(x,t,9.80665,R,W,gust,0,false);
[imu0,b0,dbg0,forceInfo0]=rocketPlantForces(x,t,9.80665,[1;2;3],R,W,gust);
[imu1,b1,dbg1,forceInfo1]=rocketPlantForces(x,t,9.80665,[1;2;3],R,W,gust,0,false);
assert(isequal([a0;alpha0;d0;info0],[a1;alpha1;d1;info1]) && ...
    isequal(dx0,dx1) && isequal(step0,step1) && ...
    isequal([imu0;b0;dbg0;forceInfo0],[imu1;b1;dbg1;forceInfo1]), ...
    'RocketCdCheck:NeutralConsumers','Disabled override changes dynamics or sensor truth.');
assert(numel(info1)==5 && info1(2)==0 && info1(3)==0 && info1(4)==info1(1) && ...
    numel(d1)==18 && isequal(forceInfo1,info1), ...
    'RocketCdCheck:NeutralInfo','Cd five-tuple or existing 18-vector is incorrect.');
[fIgnored,mIgnored]=rocketPlantAerodynamics(x,t,R,W,gust,4,false);
stepIgnored=rocketPlantStep(x,t,9.80665,R,W,gust,4,false);
assert(isequal(fIgnored,f0) && isequal(mIgnored,m0) && isequal(stepIgnored,step0), ...
    'RocketCdCheck:DisabledValue','Disabled override does not ignore a valid nonzero command.');
compact=rocketPlantStep(x,t,9.80665,R);
expanded=rocketPlantStep(x,t,9.80665,R,[],zeros(3,1),0,false);
assert(isequal(compact,expanded),'RocketCdCheck:CompactAPI','Original compact Step call changed behavior.');
metrics=struct('DisabledExactlyPreservesConsumers',true,'LegacyDebugWidth',numel(d1), ...
    'CdDiagnosticWidth',numel(info1),'NonzeroDisabledCommandIgnored',true);
end

function metrics=checkCoefficient(R,W)
R.WindNED(:)=0;
R.AeroSurfaceCant(:)=0;
W.UseProfile=0;
W.WindNED(:)=0;
W.GustEnabled=0;
x=freeState(R);
x(7:10)=[0;0;0;1];
x(11:13)=0;
[rho,~,sound]=rocketPlantAtmosphere(R.AltitudeMSL+1000);
times=[R.IgnitionTime+1;R.IgnitionTime+R.ThrustTime(end)+1];
machPoints=[0.3;0.8;1.2;3.5];
commands=[0;0.2;0.8;2];
maximumForceError=0;
for mode=1:numel(times)
    if mode==1, nominalTable=R.AeroCdPowered; else, nominalTable=R.AeroCdCoast; end
    for k=1:numel(machPoints)
        mach=machPoints(k);
        speed=mach*sound;
        x(4:6)=[0;0;speed];
        nominal=interp1(R.AeroMach,nominalTable,mach,'linear');
        q=0.5*rho*speed^2;
        for j=1:numel(commands)
            for enabled=[false true]
                command=commands(j);
                effective=nominal;
                if enabled, effective=command; end
                [force,~,aero]=rocketPlantAerodynamics(x,times(mode),R,W,zeros(3,1),command,enabled);
                expected=[0;0;-q*R.ReferenceArea*effective];
                maximumForceError=max(maximumForceError,norm(force-expected));
                assert(norm(force-expected)<2e-8 && ...
                    abs(aero.cdNominal-nominal)<1e-12 && abs(aero.cd-effective)<1e-12 && ...
                    aero.cdCommand==command && aero.cdOverrideEnabled==enabled && aero.cd>=0, ...
                    'RocketCdCheck:Coefficient','Absolute override, nominal Mach/burn curve or enable switch is incorrect.');
            end
        end
    end
end
x(4:6)=[0;0;100];
typedCommands={single(0.35),uint8(1)};
for k=1:numel(typedCommands)
    command=typedCommands{k};
    [~,~,aero]=rocketPlantAerodynamics(x,times(2),R,W,zeros(3,1),command,uint8(1));
    assert(aero.cd==double(command) && aero.cdCommand==double(command) && ...
        isequal(aero.cdOverrideEnabled,true), ...
        'RocketCdCheck:NumericType','Numeric command/enable class changes the absolute coefficient.');
end
metrics=struct('MaximumForceErrorN',maximumForceError,'MachChecked',machPoints, ...
    'AbsoluteCdChecked',commands,'EnabledZeroRemovesAxialDrag',true, ...
    'SingleAndIntegerInputsChecked',true);
end

function metrics=checkNormalIndependence(R,W)
x=freeState(R);
R.AeroSurfaceCant(2)=0.03;
gust=[1;-0.3;0.1];
t=R.IgnitionTime+1;
[f0,m0,a0]=rocketPlantAerodynamics(x,t,R,W,gust,0,false);
[f1,m1,a1]=rocketPlantAerodynamics(x,t,R,W,gust,1.1,true);
expectedDelta=[0;0;-a0.dynamicPressureCM*R.ReferenceArea*(1.1-a0.cdNominal)];
unchanged={'machCM','dynamicPressureCM','alpha','mach','speed','cna', ...
    'surfaceForce','surfaceMoment','cpWeighted','staticMargin','rollMoment'};
for k=1:numel(unchanged)
    field=unchanged{k};
    assert(isequal(a0.(field),a1.(field)), ...
        'RocketCdCheck:NormalCoupling','Body Cd command changes %s.',field);
end
assert(norm(f1-f0-expectedDelta)<1e-10 && isequal(m0,m1) && ...
    norm(a0.surfaceForce,'fro')>0 && norm(m0)>0, ...
    'RocketCdCheck:MomentCoupling','Body Cd control changes normal force or a CP/cant moment.');
metrics=struct('AxialForceChangeN',expectedDelta(3), ...
    'NormalForceNormN',norm(a0.surfaceForce,'fro'),'MomentNormNm',norm(m0), ...
    'UnchangedFields',{unchanged});
end

function metrics=checkForwarding(R,W)
x=freeState(R);
R.IMUOffset=[0.2;0.1;-0.1];
t=R.IgnitionTime+1;
g=9.80665;
gust=[0.7;0.1;-0.1];
[~,~,aero]=rocketPlantAerodynamics(x,t,R,W,gust,0,false);
command=aero.cdNominal+0.3;
deltaForce=[0;0;-aero.dynamicPressureCM*R.ReferenceArea*0.3];
[~,mass]=rocketPlantProperties(t,R);
expectedAcceleration=rocketPlantRotation(x(7:10))*deltaForce/mass;
[a0,alpha0,~,info0]=rocketPlantLoads(x,t,g,R,W,gust,0,false);
[a1,alpha1,~,info1]=rocketPlantLoads(x,t,g,R,W,gust,command,true);
dx0=rocketPlantDerivative(x,t,g,R,W,gust,0,false);
dx1=rocketPlantDerivative(x,t,g,R,W,gust,command,true);
[imu0,b0,~,forceInfo0]=rocketPlantForces(x,t,g,[21000;4000;41000],R,W,gust,0,false);
[imu1,b1,~,forceInfo1]=rocketPlantForces(x,t,g,[21000;4000;41000],R,W,gust,command,true);
assert(norm(a1-a0-expectedAcceleration)<1e-10 && isequal(alpha0,alpha1) && ...
    norm(dx1(4:6)-dx0(4:6)-expectedAcceleration)<1e-10 && ...
    isequal(dx0([1:3 7:17]),dx1([1:3 7:17])) && ...
    norm(imu1-imu0-deltaForce/mass)<1e-10 && isequal(b0,b1), ...
    'RocketCdCheck:Forwarding','Cd command/enable was dropped or transformed incorrectly in a consumer.');
assert(isequal(info0,forceInfo0) && isequal(info1,forceInfo1) && ...
    info1(2)==command && info1(3)==1 && info1(4)==command && info1(5)==1, ...
    'RocketCdCheck:InfoForwarding','Loads/Forces Cd five-tuple is inconsistent.');
metrics=struct('AccelerationChangeNEDMps2',expectedAcceleration, ...
    'SpecificForceChangeBodyMps2',deltaForce/mass,'CdInfo',info1);
end

function R=oneDimensionalRocket(R)
R.Ts=0.02;
R.MaxStep=0.002;
R.IgnitionTime=1e6;
R.PropellantMass=0;
R.PropellantInertia=zeros(3,1);
R.ThrustOffset=zeros(3,1);
R.ThrustDirection=[0;0;1];
R.WindNED(:)=0;
R.AeroSurfaceCNa(:)=0;
R.AeroFinPlanform(:)=0;
R.AeroFinLiftMultiplier(:)=0;
R.AeroRollForceScale(:)=0;
R.AeroRollDampScale(:)=0;
R.AngularDamping=zeros(3,1);
R.DrogueCdS=0;
R.MainCdS=0;
R.IMUOffset=zeros(3,1);
R.AeroEnabled=1;
R.AeroCdPowered(:)=0.3;
R.AeroCdCoast(:)=0.5;
end

function x=horizontalState(R,speed)
x=freeState(R);
x(7:10)=[0;sqrt(0.5);0;sqrt(0.5)]; % Body +z North.
x(4:6)=[speed;0;0];
x(11:13)=0;
end

function x=advance(x,start,duration,R,command,enabled)
steps=round(duration/R.Ts);
assert(abs(steps*R.Ts-duration)<1e-12,'RocketCdCheck:FixtureGrid','Duration does not match Ts.');
for k=1:steps
    x=rocketPlantStep(x,start+(k-1)*R.Ts,0,R,[],zeros(3,1),command,enabled);
end
end

function metrics=checkAnalyticTrajectories(R)
R=oneDimensionalRocket(R);
[rho,~,~]=rocketPlantAtmosphere(R.AltitudeMSL+1000);
command=1.1;
duration=0.4;
initialSpeed=100;
x=horizontalState(R,initialSpeed);
actual=advance(x,0,duration,R,command,true);
b=0.5*rho*R.ReferenceArea*command/R.DryMass;
expectedVelocity=initialSpeed/(1+b*initialSpeed*duration);
expectedPosition=log(1+b*initialSpeed*duration)/b;
coastVelocityError=abs(actual(4)-expectedVelocity);
coastPositionError=abs(actual(1)-expectedPosition);
assert(coastVelocityError<1e-8 && coastPositionError<1e-8 && ...
    norm(actual([2 3])-x([2 3]))<1e-12 && norm(actual(7:13)-x(7:13))<1e-12, ...
    'RocketCdCheck:CoastAnalytic','Controlled RK4 coast trajectory disagrees with dv/dt=-b*v^2 at constant altitude.');
nominal=advance(x,0,duration,R,command,false);
assert(actual(4)<nominal(4) && actual(1)<nominal(1), ...
    'RocketCdCheck:CoastResponse','Higher absolute Cd does not reduce forward coast speed/distance.');
R.IgnitionTime=0;
R.ThrustTime=[0;0.1;5;5.1];
R.ThrustForce=[0;500;500;0];
impulse=cumtrapz(R.ThrustTime,R.ThrustForce);
R.ThrustCumulativeImpulse=impulse/impulse(end);
initialSpeed=30;
command=0.9;
x=horizontalState(R,initialSpeed);
actual=advance(x,0.5,duration,R,command,true);
a=500/R.DryMass;
b=0.5*rho*R.ReferenceArea*command/R.DryMass;
terminal=sqrt(a/b);
rate=sqrt(a*b);
phase=atanh(initialSpeed/terminal);
expectedVelocity=terminal*tanh(rate*duration+phase);
expectedPosition=(log(cosh(rate*duration+phase))-log(cosh(phase)))/b;
poweredVelocityError=abs(actual(4)-expectedVelocity);
poweredPositionError=abs(actual(1)-expectedPosition);
assert(poweredVelocityError<1e-8 && poweredPositionError<1e-8 && ...
    norm(actual([2 3])-x([2 3]))<1e-12, ...
    'RocketCdCheck:PoweredAnalytic','Controlled powered RK4 trajectory disagrees with the constant-thrust tanh solution.');
nominal=advance(x,0.5,duration,R,command,false);
assert(actual(4)<nominal(4), ...
    'RocketCdCheck:PoweredResponse','Higher absolute Cd does not reduce powered forward speed.');
metrics=struct('CoastPositionErrorM',coastPositionError,'CoastVelocityErrorMps',coastVelocityError, ...
    'PoweredPositionErrorM',poweredPositionError,'PoweredVelocityErrorMps',poweredVelocityError, ...
    'DurationS',duration,'CoastCdCommand',1.1,'PoweredCdCommand',0.9);
end

function metrics=checkCommandBoundary(R)
R=oneDimensionalRocket(R);
[rho,~,~]=rocketPlantAtmosphere(R.AltitudeMSL+1000);
x=horizontalState(R,100);
actual=x;
expectedPosition=0;
expectedSpeed=100;
commands=[0.5;1.3;0];
enables=[false;true;true];
segmentDuration=0.1;
for k=1:numel(commands)
    actual=advance(actual,(k-1)*segmentDuration,segmentDuration,R,commands(k),enables(k));
    effective=0.5;
    if enables(k), effective=commands(k); end
    b=0.5*rho*R.ReferenceArea*effective/R.DryMass;
    if b>0
        expectedPosition=expectedPosition+log(1+b*expectedSpeed*segmentDuration)/b;
        expectedSpeed=expectedSpeed/(1+b*expectedSpeed*segmentDuration);
    else
        expectedPosition=expectedPosition+expectedSpeed*segmentDuration;
    end
end
assert(abs(actual(1)-expectedPosition)<1e-8 && abs(actual(4)-expectedSpeed)<1e-8, ...
    'RocketCdCheck:PiecewiseCommand','Major-step command/enable changes were delayed, smeared or omitted from RK4 stages.');
tiny=R;
tiny.Ts=1e-6;
tiny.MaxStep=tiny.Ts;
initialDerivative=rocketPlantDerivative(x,0,0,tiny,[],zeros(3,1),1.3,true);
after=rocketPlantStep(x,0,0,tiny,[],zeros(3,1),1.3,true);
continuityError=norm(after(4:6)-x(4:6)-tiny.Ts*initialDerivative(4:6));
assert(continuityError<1e-9 && norm(after(7:17)-x(7:17))<1e-12, ...
    'RocketCdCheck:Continuity','Drag control change injects an instantaneous velocity/attitude/event reset.');
metrics=struct('PiecewisePositionErrorM',abs(actual(1)-expectedPosition), ...
    'PiecewiseVelocityErrorMps',abs(actual(4)-expectedSpeed), ...
    'OnsetVelocityContinuityErrorMps',continuityError,'Commands',commands,'Enables',enables);
end

function metrics=checkParachutes(R,W)
x=freeState(R);
x(11:13)=0;
x(14)=3;
x(15)=R.IgnitionTime+5;
x(16)=R.IgnitionTime+10;
x(17)=-1;
times=x(16)+R.DrogueInflationTime*[0.5;1.1];
scales=zeros(2,1);
for k=1:2
    time=times(k);
    [~,~,aero]=rocketPlantAerodynamics(x,time,R,W,zeros(3,1),0,false);
    command=aero.cdNominal+0.4;
    [a0,alpha0,d0,info0]=rocketPlantLoads(x,time,9.80665,R,W,zeros(3,1),0,false);
    [a1,alpha1,d1,info1]=rocketPlantLoads(x,time,9.80665,R,W,zeros(3,1),command,true);
    scales(k)=info1(5);
    assert(isequal(d0(9:10),d1(9:10)) && isequal(alpha0,alpha1) && info0(5)==info1(5), ...
        'RocketCdCheck:CanopyCoupling','Body Cd control changes canopy CdS, inflation or moment.');
    [~,mass]=rocketPlantProperties(time,R);
    expected=rocketPlantRotation(x(7:10))* ...
        [0;0;-aero.dynamicPressureCM*R.ReferenceArea*0.4*info1(5)]/mass;
    assert(norm(a1-a0-expected)<1e-10, ...
        'RocketCdCheck:RecoveryFade','Cd override does not respect the body aerodynamic recovery fade.');
    if k==2
        assert(isequal(a0,a1) && info1(5)==0, ...
            'RocketCdCheck:FullyInflated','Body Cd affects fully inflated drogue descent.');
    end
end
assert(abs(scales(1)-0.5)<1e-12,'RocketCdCheck:PartialInflation','Mid-inflation fixture does not have half body aero.');
x(14)=4;
x(17)=x(16)+5;
time=x(17)+1.1*R.MainInflationTime;
[a0,alpha0,d0,info0]=rocketPlantLoads(x,time,9.80665,R,W,zeros(3,1),0,false);
[a1,alpha1,d1,info1]=rocketPlantLoads(x,time,9.80665,R,W,zeros(3,1),2,true);
assert(d0(9)>0 && d0(10)>0 && isequal(d0(9:10),d1(9:10)) && ...
    isequal([a0;alpha0],[a1;alpha1]) && info0(5)==0 && info1(5)==0, ...
    'RocketCdCheck:MainCanopyCoupling','Absolute body Cd changes either fully inflated canopy or main-descent loads.');
disabled=R;
disabled.AeroEnabled=0;
x(14)=3;
x(17)=-1;
[a0,alpha0,~,info0]=rocketPlantLoads(x,times(1),9.80665,disabled,W,zeros(3,1),0,false);
[a1,alpha1,~,info1]=rocketPlantLoads(x,times(1),9.80665,disabled,W,zeros(3,1),2,true);
assert(isequal([a0;alpha0],[a1;alpha1]) && info0(5)==0 && info1(5)==0, ...
    'RocketCdCheck:DisabledBody','Disabled body aero still responds to Cd command.');
metrics=struct('PartialBodyScale',scales(1),'FullyInflatedBodyScale',scales(2), ...
    'BothCanopyCdSUnchanged',true,'DisabledBodyControlInactive',true);
end

function metrics=checkInvalid(R,W)
x=freeState(R);
t=R.IgnitionTime+1;
gust=zeros(3,1);
commands={NaN,Inf,-Inf,-0.1,[0;1],[],1+1i,'0',struct('cd',1)};
commandNames={'NaN','Inf','negativeInf','negative','vector','empty','complex','text','structure'};
enables={NaN,Inf,-1,2,[0;1],[],1+1i,'true',struct('enabled',true)};
enableNames={'enableNaN','enableInf','enableNegative','enableTwo','enableVector', ...
    'enableEmpty','enableComplex','enableText','enableStructure'};
consumers={@(command,enabled) rocketPlantAerodynamics(x,t,R,W,gust,command,enabled), ...
    @(command,enabled) rocketPlantLoads(x,t,9.80665,R,W,gust,command,enabled), ...
    @(command,enabled) rocketPlantDerivative(x,t,9.80665,R,W,gust,command,enabled), ...
    @(command,enabled) rocketPlantStep(x,t,9.80665,R,W,gust,command,enabled), ...
    @(command,enabled) rocketPlantForces(x,t,9.80665,[1;2;3],R,W,gust,command,enabled)};
for consumer=1:numel(consumers)
    for k=1:numel(commands)
        requireInvalid(@() consumers{consumer}(commands{k},true),commandNames{k});
        requireInvalid(@() consumers{consumer}(commands{k},false),commandNames{k});
    end
    for k=1:numel(enables)
        requireInvalid(@() consumers{consumer}(0,enables{k}),enableNames{k});
    end
end
R.AeroEnabled=0;
requireInvalid(@() rocketPlantAerodynamics(x,t,R,W,gust,NaN,false),'disabledAeroNaN');
x(14)=0;
requireInvalid(@() rocketPlantDerivative(x,t,9.80665,R,W,gust,-1,false),'padNegativeCommand');
requireInvalid(@() rocketPlantStep(x,t,9.80665,R,W,gust,0,2),'padInvalidEnable');
metrics=struct('InvalidCommandsChecked',{commandNames},'InvalidEnablesChecked',{enableNames}, ...
    'ConsumerCount',numel(consumers),'RejectedIdentifier','RocketPlant:InvalidCdControl', ...
    'DisabledAeroAndPadValidated',true);
end

function requireInvalid(call,name)
identifier='';
try
    call();
catch exception
    identifier=exception.identifier;
end
assert(strcmp(identifier,'RocketPlant:InvalidCdControl'), ...
    'RocketCdCheck:InvalidAccepted','Invalid Cd case %s returned error %s instead of RocketPlant:InvalidCdControl.',name,identifier);
end
