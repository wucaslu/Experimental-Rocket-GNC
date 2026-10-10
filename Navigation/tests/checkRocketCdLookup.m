function report = checkRocketCdLookup(outputDirectory)
%CHECKROCKETCDLOOKUP Speed/control Cd interpolation and plant propagation.
% Uses a nonuniform grid with an independent bilinear polynomial oracle,
% checks source priority and wind frames, then checks sampled-Cd dynamics
% against the closed-form quadratic-drag solution. No model is opened or
% changed. The editable example CSV is loaded through its settings script.
if nargin<1, outputDirectory=''; end
navigationDirectory=fileparts(fileparts(mfilename('fullpath')));
previousPath=path;
pathCleanup=onCleanup(@() path(previousPath));
addpath(fullfile(navigationDirectory,'functions','aux_Rocket'));
[Rocket,example]=loadSettings(navigationDirectory);
names={'nonuniformBilinearOracle','clippingAndNumericScalars', ...
    'editableExampleTable','windFrameAndSourcePriority', ...
    'forceSampleHoldAndContinuity','invalidInputsAndTables'};
checks={@checkBilinear,@checkClipping,@() checkExample(example), ...
    @checkSelection,@() checkDynamics(Rocket),@checkInvalid};
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
    filename=fullfile(outputDirectory,'rocket-cd-lookup-checks.json');
    file=fopen(filename,'w');
    assert(file>=0,'RocketCdLookupCheck:ReportFile','Cannot write %s.',filename);
    fileCleanup=onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(report,PrettyPrint=true));
end
assert(report.Passed,'RocketCdLookupCheck:Failed','Cd lookup checks failed: %s.', ...
    strjoin({results(~[results.Passed]).Name},', '));
end

function [Rocket,CdLookup]=loadSettings(navigationDirectory)
Rocket=struct();
CdLookup=struct();
run(fullfile(navigationDirectory,'settings','rocketPlantSettings.m'));
run(fullfile(navigationDirectory,'settings','rocketCdLookupSettings.m'));
end

function lookup=polynomialTable()
lookup=struct('VelocityBreakpoints',[5;37;118;257], ...
    'ControlBreakpoints',0:0.1:1,'Table',zeros(4,11));
[speed,control]=ndgrid(lookup.VelocityBreakpoints,lookup.ControlBreakpoints);
lookup.Table=polynomialCd(speed,control);
end

function cd=polynomialCd(speed,control)
% A cross term exercises both interpolation dimensions without reproducing
% the interval search or four-corner blend used by the implementation.
cd=0.23+0.0017*speed+0.31*control+0.0009*speed.*control;
end

function metrics=checkBilinear()
lookup=polynomialTable();
queries=[5 0;37 0.1;118 0.4;257 1;5 1;257 0;118 0.7; ...
    19 0.035;67 0.46;211 0.83];
maximumError=0;
for k=1:size(queries,1)
    [actual,used]=rocketPlantCdLookup(queries(k,1),queries(k,2),lookup);
    expected=polynomialCd(queries(k,1),queries(k,2));
    maximumError=max(maximumError,abs(actual-expected));
    assert(abs(actual-expected)<1e-12 && norm(used-queries(k,:).')<1e-12, ...
        'RocketCdLookupCheck:Bilinear', ...
        'Cd or query differs from the independent polynomial at speed %g, control %g.', ...
        queries(k,1),queries(k,2));
end
metrics=struct('VelocityBreakpointsMps',lookup.VelocityBreakpoints, ...
    'Queries',queries,'MaximumOracleError',maximumError, ...
    'ControlInteriorIsInterpolated',true);
end

function metrics=checkClipping()
lookup=polynomialTable();
queries=[0 -0.4;400 1.5;19 1.5;400 0.35;0 0.65];
expectedQueries=[5 0;257 1;19 1;257 0.35;5 0.65];
maximumError=0;
for k=1:size(queries,1)
    [actual,used]=rocketPlantCdLookup(queries(k,1),queries(k,2),lookup);
    expected=polynomialCd(expectedQueries(k,1),expectedQueries(k,2));
    maximumError=max(maximumError,abs(actual-expected));
    assert(norm(used-expectedQueries(k,:).')<1e-12 && ...
        abs(actual-expected)<1e-12 && actual>=0, ...
        'RocketCdLookupCheck:Clipping','Finite out-of-range queries were extrapolated or clipped incorrectly.');
end
speeds={uint16(37),single(67.5)};
controls={uint8(1),single(0.45)};
for k=1:numel(speeds)
    [actual,used]=rocketPlantCdLookup(speeds{k},controls{k},lookup);
    expectedQuery=[double(speeds{k});double(controls{k})];
    expected=polynomialCd(expectedQuery(1),expectedQuery(2));
    assert(isa(actual,'double') && isa(used,'double') && ...
        norm(used-expectedQuery)<1e-12 && abs(actual-expected)<1e-12, ...
        'RocketCdLookupCheck:NumericScalar','Single/integer scalars change interpolation or output class.');
end
metrics=struct('ClippedQueries',expectedQueries,'MaximumOracleError',maximumError, ...
    'SingleAndIntegerScalarsChecked',true,'NonnegativeResults',true);
end

function metrics=checkExample(lookup)
rocketPlantValidateCdLookup(lookup);
assert(iscolumn(lookup.VelocityBreakpoints) && numel(lookup.VelocityBreakpoints)>=2 && ...
    all(diff(lookup.VelocityBreakpoints)>0) && all(lookup.VelocityBreakpoints>=0) && ...
    isequal(size(lookup.Table),[numel(lookup.VelocityBreakpoints),11]) && ...
    norm(lookup.ControlBreakpoints-(0:0.1:1))<1e-12 && ...
    all(isfinite(lookup.Table(:))) && all(lookup.Table(:)>=0), ...
    'RocketCdLookupCheck:ExampleGrid','The editable settings do not load a complete nonnegative 11-level table.');
maximumNodeError=0;
for column=1:11
    row=1+mod(column-1,numel(lookup.VelocityBreakpoints));
    actual=rocketPlantCdLookup(lookup.VelocityBreakpoints(row), ...
        lookup.ControlBreakpoints(column),lookup);
    maximumNodeError=max(maximumNodeError,abs(actual-lookup.Table(row,column)));
end
assert(maximumNodeError<1e-12,'RocketCdLookupCheck:ExampleNodes', ...
    'Loaded table node values do not survive lookup at all eleven control levels.');
metrics=struct('VelocityRows',numel(lookup.VelocityBreakpoints),'ControlLevels',11, ...
    'SpeedRangeMps',[lookup.VelocityBreakpoints(1);lookup.VelocityBreakpoints(end)], ...
    'CdRange',[min(lookup.Table(:));max(lookup.Table(:))], ...
    'MaximumNodeError',maximumNodeError);
end

function metrics=checkSelection()
lookup=polynomialTable();
velocityNEU=[30;-4;12];
windNED=[6;-4;-2];
speed=26; % norm([30;-4;-12]-[6;-4;-2]) = norm([24;0;-10]).
control=0.35;
expectedLookup=polynomialCd(speed,control);
[nominalCd,nominalEnabled,nominalInfo]=rocketPlantCdLookupCommand( ...
    velocityNEU,windNED,control,false,0.77,false,lookup);
[lookupCd,lookupEnabled,lookupInfo]=rocketPlantCdLookupCommand( ...
    velocityNEU,windNED,control,uint8(1),0.77,uint8(0),lookup);
[manualCd,manualEnabled,manualInfo]=rocketPlantCdLookupCommand( ...
    velocityNEU,windNED,control,true,2.3,true,lookup);
[manualOnlyCd,manualOnlyEnabled,manualOnlyInfo]=rocketPlantCdLookupCommand( ...
    velocityNEU,windNED,control,false,2.3,true,lookup);
expectedBase=[speed;speed;control;control;expectedLookup];
assert(nominalCd==0.77 && ~nominalEnabled && ...
    abs(lookupCd-expectedLookup)<1e-12 && lookupEnabled && ...
    manualCd==2.3 && manualEnabled && manualOnlyCd==2.3 && manualOnlyEnabled, ...
    'RocketCdLookupCheck:Priority','Nominal/lookup/manual selection has incorrect priority or disabled command diagnostics.');
assert(norm(nominalInfo-[expectedBase;0;0])<1e-12 && ...
    norm(lookupInfo-[expectedBase;1;1])<1e-12 && ...
    norm(manualInfo-[expectedBase;1;2])<1e-12 && ...
    norm(manualOnlyInfo-[expectedBase;0;2])<1e-12, ...
    'RocketCdLookupCheck:WindFrame','NEU-to-NED wind subtraction or source diagnostics are incorrect.');
[~,~,clippedInfo]=rocketPlantCdLookupCommand( ...
    velocityNEU,windNED,1.7,true,0,false,lookup);
assert(abs(clippedInfo(3)-1.7)<1e-12 && clippedInfo(4)==1 && ...
    abs(clippedInfo(5)-polynomialCd(speed,1))<1e-12, ...
    'RocketCdLookupCheck:RawControl','Clipping hides the raw control diagnostic or changes the boundary Cd.');
metrics=struct('AirVelocityNEDMps',[24;0;-10],'AirspeedMps',speed, ...
    'SelectedLookupCd',lookupCd,'ManualPriorityCd',manualCd, ...
    'NominalDisabledCommand',nominalCd,'SourceModes',[nominalInfo(7);lookupInfo(7);manualInfo(7)]);
end

function R=horizontalCoastRocket(R)
R.IgnitionTime=1e6;
R.PropellantMass=0;
R.PropellantInertia=zeros(3,1);
R.ThrustOffset=zeros(3,1);
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
end

function metrics=checkDynamics(R)
R=horizontalCoastRocket(R);
lookup=polynomialTable();
x=rocketPlantInitialize(R);
x(1:3)=[0;0;-1000];
x(4:6)=[100;0;0];
x(7:10)=[0;sqrt(0.5);0;sqrt(0.5)]; % Body +Z North.
x(11:13)=0;
x(14)=2;
[lowCd,lowEnabled]=rocketPlantCdLookupCommand([100;0;0],zeros(3,1),0.1,true,0,false,lookup);
[highCd,highEnabled]=rocketPlantCdLookupCommand([100;0;0],zeros(3,1),0.9,true,0,false,lookup);
[rho,~,~]=rocketPlantAtmosphere(R.AltitudeMSL+1000);
pressure=0.5*rho*100^2;
expectedForceChange=[0;0;-pressure*R.ReferenceArea*(highCd-lowCd)/R.DryMass];
[lowForce,lowMagnetic,~,lowInfo]=rocketPlantForces(x,0,0,[1;2;3],R,[],zeros(3,1),lowCd,lowEnabled);
[highForce,highMagnetic,~,highInfo]=rocketPlantForces(x,0,0,[1;2;3],R,[],zeros(3,1),highCd,highEnabled);
assert(highCd>lowCd && norm(highForce-lowForce-expectedForceChange)<1e-10 && ...
    isequal(lowMagnetic,highMagnetic) && abs(lowInfo(4)-lowCd)<1e-12 && ...
    abs(highInfo(4)-highCd)<1e-12, ...
    'RocketCdLookupCheck:ForcePropagation','Changed control at the same speed does not produce the expected axial drag and IMU force.');
low=rocketPlantStep(x,0,0,R,[],zeros(3,1),lowCd,lowEnabled);
high=rocketPlantStep(x,0,0,R,[],zeros(3,1),highCd,highEnabled);
bLow=0.5*rho*R.ReferenceArea*lowCd/R.DryMass;
bHigh=0.5*rho*R.ReferenceArea*highCd/R.DryMass;
expectedLow=[log1p(bLow*100*R.Ts)/bLow;100/(1+bLow*100*R.Ts)];
expectedHigh=[log1p(bHigh*100*R.Ts)/bHigh;100/(1+bHigh*100*R.Ts)];
holdError=max(norm(low([1 4])-expectedLow),norm(high([1 4])-expectedHigh));
assert(holdError<1e-8 && high(4)<low(4) && high(1)<low(1), ...
    'RocketCdLookupCheck:SampleHold','Lookup Cd was dropped or changed within the sampled quadratic-drag step.');
[disabledCd,disabledEnabled]=rocketPlantCdLookupCommand([100;0;0],zeros(3,1),0.9,false,0.77,false,lookup);
disabled=rocketPlantStep(x,0,0,R,[],zeros(3,1),disabledCd,disabledEnabled);
nominal=rocketPlantStep(x,0,0,R);
assert(isequal(disabled,nominal),'RocketCdLookupCheck:NominalCompatibility', ...
    'Disabling both sources changes the nominal plant state.');
tiny=R;
tiny.Ts=1e-6;
tiny.MaxStep=tiny.Ts;
velocityNEU=low(4:6).*[1;1;-1];
[switchedCd,switchedEnabled]=rocketPlantCdLookupCommand(velocityNEU,zeros(3,1),0.9,true,0,false,lookup);
afterSwitch=rocketPlantStep(low,R.Ts,0,tiny,[],zeros(3,1),switchedCd,switchedEnabled);
bSwitch=0.5*rho*R.ReferenceArea*switchedCd/R.DryMass;
expectedSwitchSpeed=low(4)/(1+bSwitch*low(4)*tiny.Ts);
assert(abs(afterSwitch(4)-expectedSwitchSpeed)<1e-9 && ...
    norm(afterSwitch(4:6)-low(4:6))<1e-3 && ...
    norm(afterSwitch(7:17)-low(7:17))<1e-12, ...
    'RocketCdLookupCheck:Continuity','A control-level change resets velocity, attitude or an event state.');
metrics=struct('LowControl',0.1,'HighControl',0.9,'LowCd',lowCd,'HighCd',highCd, ...
    'SpecificForceChangeBodyMps2',expectedForceChange,'MaximumHeldStepOracleError',holdError, ...
    'SwitchedVelocityOracleErrorMps',abs(afterSwitch(4)-expectedSwitchSpeed), ...
    'NominalExactlyPreserved',true,'ControlChangePreservesStateContinuity',true);
end

function metrics=checkInvalid()
lookup=polynomialTable();
inputId='RocketPlant:InvalidCdLookupInput';
tableId='RocketPlant:InvalidCdLookupTable';
controlId='RocketPlant:InvalidCdControl';
badSpeeds={-1,NaN,Inf,[1;2],[],1+1i,'10'};
badControls={NaN,Inf,[0 0.1],[],0.5+1i,'0.5'};
for k=1:numel(badSpeeds)
    requireError(@() rocketPlantCdLookup(badSpeeds{k},0.5,lookup),inputId,'speed');
end
for k=1:numel(badControls)
    requireError(@() rocketPlantCdLookup(50,badControls{k},lookup),inputId,'control');
end
badTables=repmat({lookup},1,12);
badTables{1}=rmfield(lookup,'Table');
badTables{2}.VelocityBreakpoints=lookup.VelocityBreakpoints.';
badTables{3}.VelocityBreakpoints=[5;37;37;257];
badTables{4}.VelocityBreakpoints=[-1;37;118;257];
badTables{5}.VelocityBreakpoints=[5;NaN;118;257];
badTables{6}.VelocityBreakpoints=5;
badTables{7}.ControlBreakpoints=lookup.ControlBreakpoints.';
badTables{8}.ControlBreakpoints=lookup.ControlBreakpoints+0.001;
badTables{9}.Table=lookup.Table.';
badTables{10}.Table=lookup.Table(:,1:10);
badTables{11}.Table(2,3)=-0.1;
badTables{12}.Table(2,3)=NaN;
tableCases={'missingTable','velocityRow','duplicateSpeed','negativeSpeed', ...
    'nonfiniteSpeed','singleSpeed','controlColumn','controlGridOffset', ...
    'transposedSurface','missingControlColumn','negativeCd','nonfiniteCd'};
for k=1:numel(badTables)
    requireError(@() rocketPlantCdLookup(50,0.5,badTables{k}),tableId,tableCases{k});
end
requireError(@() rocketPlantCdLookup(50,0.5,[]),tableId,'nonstructTable');
requireError(@() rocketPlantCdLookupCommand([1 2 3],zeros(3,1),0,false,0,false,lookup),inputId,'velocityRow');
requireError(@() rocketPlantCdLookupCommand([1;2;NaN],zeros(3,1),0,false,0,false,lookup),inputId,'ignoredNonfiniteVelocity');
requireError(@() rocketPlantCdLookupCommand(zeros(3,1),[1 2 3],0,false,0,false,lookup),inputId,'windRow');
requireError(@() rocketPlantCdLookupCommand(zeros(3,1),[0;Inf;0],0,false,0,false,lookup),inputId,'ignoredNonfiniteWind');
requireError(@() rocketPlantCdLookupCommand(zeros(3,1),zeros(3,1),NaN,false,1,true,lookup),inputId,'manualPriorityInvalidControl');
requireError(@() rocketPlantCdLookupCommand(zeros(3,1),zeros(3,1),0,false,1,true,badTables{8}),tableId,'manualPriorityInvalidTable');
requireError(@() rocketPlantCdLookupCommand(zeros(3,1),zeros(3,1),0,true,-1,false,lookup),controlId,'ignoredNegativeManual');
requireError(@() rocketPlantCdLookupCommand(zeros(3,1),zeros(3,1),0,false,NaN,false,lookup),controlId,'ignoredNonfiniteManual');
requireError(@() rocketPlantCdLookupCommand(zeros(3,1),zeros(3,1),0,2,1,true,lookup),controlId,'manualPriorityInvalidLookupEnable');
requireError(@() rocketPlantCdLookupCommand(zeros(3,1),zeros(3,1),0,false,0,2,lookup),controlId,'invalidManualEnable');
metrics=struct('InvalidScalarQueriesChecked',numel(badSpeeds)+numel(badControls), ...
    'InvalidTableCases',{tableCases},'InvalidSelectorCasesChecked',10, ...
    'RejectedIdentifiers',{{inputId,tableId,controlId}},'IgnoredRawInputsStillValidated',true);
end

function requireError(call,expectedIdentifier,name)
identifier='';
try
    call();
catch exception
    identifier=exception.identifier;
end
assert(strcmp(identifier,expectedIdentifier),'RocketCdLookupCheck:InvalidAccepted', ...
    'Invalid %s returned error %s instead of %s.',name,identifier,expectedIdentifier);
end
