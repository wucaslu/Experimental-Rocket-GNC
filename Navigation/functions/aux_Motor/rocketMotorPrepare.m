function [StaticFire,typeCode,Properties,Info] = rocketMotorPrepare(source,motorType,properties,options)
%ROCKETMOTORPREPARE Validate static-fire data and prepare numeric motor inputs.
% INPUTS
%   source: numeric N-by-2 [time_s,thrust_N], a CSV filename, or a table
%       containing time_s/thrust_N. Optional table/CSV columns are
%       fuel_mass_kg and oxidizer_mass_kg; solid motors may instead use
%       propellant_mass_kg as an unambiguous alias for fuel_mass_kg.
%   motorType: 'solid', 'hybrid', 'liquid', or numeric codes 1, 2, 3.
%   properties: scalar structure with DryMass [kg], DryInertia [3x1 kg*m^2],
%       FuelInitialMass/FuelResidualMass [kg], OxidizerInitialMass/
%       OxidizerResidualMass [kg], and FuelInitialInertia/FuelResidualInertia/
%       OxidizerInitialInertia/OxidizerResidualInertia [3x1 kg*m^2].
%       Every inertia is intrinsic about that component's own center of mass.
%       Optional DryCenterOfMassZ, FuelInitialCenterOfMassZ,
%       FuelResidualCenterOfMassZ, OxidizerInitialCenterOfMassZ and
%       OxidizerResidualCenterOfMassZ are axial positions [m] in the common
%       body-fixed reference frame. Each omitted position defaults to zero.
%       Optional GrainInertiaEnabled=1 enables symmetric BATES geometry for
%       solid motors. Supply GrainCount, GrainDensity [kg/m^3],
%       GrainOuterRadius, GrainInitialInnerRadius, GrainInitialHeight and
%       GrainSeparation [m]. Fuel initial/residual inertias are then derived
%       from geometry; the fuel centroid must be fixed. Defaults disable it.
%   options: optional scalar initialization structure. TimeColumn and
%       ThrustColumn select table/CSV headers (defaults time_s/thrust_N);
%       values must already be seconds/newtons. EndpointMode='zero' requires
%       zero thrust at both endpoints. Explicit EndpointMode='abrupt' preserves
%       nonzero endpoint measurements and applies zero thrust outside the
%       record, with burnout at its final timestamp. No tail is synthesized.
%       Optional BurnOutTime [s] clips the record to that positive timestamp,
%       interpolating a boundary sample without rescaling time or thrust.
% OUTPUTS
%   StaticFire: numeric curve, physical cumulative/total impulse [N*s],
%       normalized remaining channel fractions, and numeric profile flags.
%   typeCode: 1 solid, 2 hybrid, 3 liquid.
%   Properties: validated double-valued runtime property structure.
%   Info: initialization-only provenance and explicit modeling assumptions.
% This initializer does file I/O when requested. The runtime model does none.
% Without measured mass profiles, mass flow follows measured thrust assuming
% constant effective exhaust velocity; this is not a combustion prediction.

if nargin<4,options=struct();end
options=importOptions(options);
typeCode = parseType(motorType);
typeNames = {'solid','hybrid','liquid'};
Properties = validateProperties(properties,typeCode);
hasFuelProfile = false;
hasOxidizerProfile = false;
fuelMass = zeros(0,1);
oxidizerMass = zeros(0,1);

if isnumeric(source)
    assert(isreal(source) && ismatrix(source) && size(source,2)==2 && ...
        size(source,1)>=2 && all(isfinite(source(:))), ...
        'RocketMotor:NumericSource','Numeric source must be finite real N-by-2 [time_s,thrust_N].');
    time = full(double(source(:,1)));
    thrust = full(double(source(:,2)));
    sourceDescription = 'Numeric time/thrust matrix';
else
    if ischar(source) || (isstring(source) && isscalar(source))
        assert((ischar(source) && isrow(source)) || isstring(source), ...
            'RocketMotor:SourceFilename','CSV source must be one filename.');
        sourceDescription = char(source);
        assert(isfile(sourceDescription),'RocketMotor:MissingSource', ...
            'Static-fire CSV does not exist: %s.',sourceDescription);
        [~,~,extension] = fileparts(sourceDescription);
        assert(strcmpi(extension,'.csv'),'RocketMotor:SourceExtension', ...
            'The file source must be a CSV; other formats are not inferred.');
        data = readtable(sourceDescription,'VariableNamingRule','preserve','TextType','string');
    else
        assert(istable(source),'RocketMotor:SourceType', ...
            'Source must be a numeric N-by-2 matrix, CSV filename, or table.');
        data = source;
        sourceDescription = 'MATLAB static-fire table';
    end
    names = data.Properties.VariableNames;
    required = {options.TimeColumn,options.ThrustColumn};
    allowed = [required,{'fuel_mass_kg','oxidizer_mass_kg','propellant_mass_kg'}];
    assert(all(ismember(required,names)),'RocketMotor:MissingColumns', ...
        'Static-fire tables require %s and %s columns.',required{1},required{2});
    assert(all(ismember(names,allowed)),'RocketMotor:UnknownColumns', ...
        'Unsupported static-fire columns: %s.',strjoin(names(~ismember(names,allowed)),', '));
    assert(height(data)>=2,'RocketMotor:CurveLength','At least two samples are required.');
    time = numericColumn(data,options.TimeColumn);
    thrust = numericColumn(data,options.ThrustColumn);
    hasFuelColumn = ismember('fuel_mass_kg',names);
    hasAlias = ismember('propellant_mass_kg',names);
    assert(~hasAlias || typeCode==1,'RocketMotor:PropellantAliasType', ...
        'propellant_mass_kg is only supported for a solid motor.');
    assert(~(hasFuelColumn && hasAlias),'RocketMotor:DuplicateFuelColumns', ...
        'Use fuel_mass_kg or propellant_mass_kg, not both.');
    hasFuelProfile = hasFuelColumn || hasAlias;
    hasOxidizerProfile = ismember('oxidizer_mass_kg',names);
    if hasFuelColumn
        fuelMass = numericColumn(data,'fuel_mass_kg');
    elseif hasAlias
        fuelMass = numericColumn(data,'propellant_mass_kg');
    end
    if hasOxidizerProfile
        oxidizerMass = numericColumn(data,'oxidizer_mass_kg');
    end
end

assert(time(1)==0 && all(diff(time)>0),'RocketMotor:TimeGrid', ...
    'Curve time must start at zero and increase strictly.');
assert(all(thrust>=0),'RocketMotor:ThrustCurve','Thrust must be nonnegative.');
sourceSampleCount=numel(time);
sourceDuration=time(end);
sourceImpulse=trapz(time,thrust);
if ~isempty(options.BurnOutTime)
    assert(options.BurnOutTime<=sourceDuration,'RocketMotor:BurnOutTime', ...
        'BurnOutTime cannot exceed the measured record; extrapolated tails are not supplied.');
    if options.BurnOutTime<sourceDuration
        retained=time<options.BurnOutTime;
        boundaryThrust=interp1(time,thrust,options.BurnOutTime,'linear');
        if hasFuelProfile
            boundaryFuel=interp1(time,fuelMass,options.BurnOutTime,'linear');
            fuelMass=[fuelMass(retained);boundaryFuel];
        end
        if hasOxidizerProfile
            boundaryOxidizer=interp1(time,oxidizerMass,options.BurnOutTime,'linear');
            oxidizerMass=[oxidizerMass(retained);boundaryOxidizer];
        end
        time=[time(retained);options.BurnOutTime];
        thrust=[thrust(retained);boundaryThrust];
    end
end
if strcmp(options.EndpointMode,'zero')
    assert(thrust(1)==0 && thrust(end)==0,'RocketMotor:ThrustCurve', ...
        'Zero-force endpoints are required unless EndpointMode=abrupt is explicitly selected.');
end
cumulativeImpulse = cumtrapz(time,thrust);
totalImpulse = cumulativeImpulse(end);
assert(isfinite(totalImpulse) && totalImpulse>0,'RocketMotor:TotalImpulse', ...
    'The curve must have finite positive total impulse.');
fallbackFraction = 1-cumulativeImpulse/totalImpulse;
fuelConsumed = Properties.FuelInitialMass-Properties.FuelResidualMass;
oxidizerConsumed = Properties.OxidizerInitialMass-Properties.OxidizerResidualMass;
if hasFuelProfile
    fuelFraction = profileFraction(fuelMass,Properties.FuelInitialMass, ...
        Properties.FuelResidualMass,'fuel');
else
    fuelFraction = fallbackFraction;
end
if hasOxidizerProfile
    oxidizerFraction = profileFraction(oxidizerMass,Properties.OxidizerInitialMass, ...
        Properties.OxidizerResidualMass,'oxidizer');
else
    oxidizerFraction = fallbackFraction;
end
StaticFire = struct('Time',time,'Thrust',thrust, ...
    'CumulativeImpulse',cumulativeImpulse,'TotalImpulse',totalImpulse, ...
    'FuelFraction',fuelFraction,'OxidizerFraction',oxidizerFraction, ...
    'HasFuelMassProfile',double(hasFuelProfile), ...
    'HasOxidizerMassProfile',double(hasOxidizerProfile));
if strcmp(options.EndpointMode,'abrupt')
    % Numeric provenance also survives cached validation and Monte Carlo
    % re-preparation; legacy zero-endpoint runtime structures are unchanged.
    StaticFire.AbruptEndpoints=1;
end
constantRatio = typeCode>1 && ~hasFuelProfile && ~hasOxidizerProfile;
if fuelConsumed>0
    consumedRatio = oxidizerConsumed/fuelConsumed;
else
    consumedRatio = 0;
end
Info = struct('MotorType',typeNames{typeCode},'TypeCode',typeCode, ...
    'Source',sourceDescription,'SampleCount',numel(time), ...
    'BurnDurationS',time(end),'TotalImpulseNs',totalImpulse, ...
    'FuelConsumedMassKg',fuelConsumed,'OxidizerConsumedMassKg',oxidizerConsumed, ...
    'FuelMassProfileSupplied',hasFuelProfile, ...
    'OxidizerMassProfileSupplied',hasOxidizerProfile, ...
    'ConstantConsumedMixtureRatioAssumed',constantRatio, ...
    'ConsumedOxidizerFuelRatio',consumedRatio, ...
    'FuelMassMethod',massMethod(hasFuelProfile,fuelConsumed), ...
    'OxidizerMassMethod',massMethod(hasOxidizerProfile,oxidizerConsumed), ...
    'SolidGrainGeometryEnabled',Properties.GrainInertiaEnabled>0.5, ...
    'FuelInertiaMethod',fuelInertiaMethod(Properties.GrainInertiaEnabled), ...
    'Assumptions',{{'Absent mass profiles use thrust-proportional mass flow with constant effective exhaust velocity.', ...
        'With both hybrid/liquid profiles absent, consumed oxidizer/fuel flow ratio is constant.', ...
        'Enabled solid BATES inertia follows symmetric bore/end regression reconstructed from remaining mass; other channel inertias and axial centers use endpoint interpolation.', ...
        'Component centers use one body-fixed reference; aggregate motor inertia includes parallel-axis shifts to its moving center of mass.', ...
        'Static-fire thrust is replayed; grain geometry follows mass depletion, while chamber pressure, combustion and altitude thrust correction are not predicted.'}});
Info.EndpointMode=options.EndpointMode;
Info.TimeColumn=options.TimeColumn;
Info.ThrustColumn=options.ThrustColumn;
Info.SourceSampleCount=sourceSampleCount;
Info.SourceDurationS=sourceDuration;
Info.SourceTotalImpulseNs=sourceImpulse;
Info.ExcludedImpulseNs=sourceImpulse-totalImpulse;
if strcmp(options.EndpointMode,'abrupt')
    Info.Assumptions{end+1}= ...
        'Explicit abrupt endpoints preserve all recorded thrust values; thrust jumps to zero and configured residual propellant is reached at the final timestamp.';
end
end

function options=importOptions(provided)
assert(isstruct(provided) && isscalar(provided),'RocketMotor:ImportOptions', ...
    'Import options must be a scalar structure.');
options=struct('TimeColumn','time_s','ThrustColumn','thrust_N', ...
    'EndpointMode','zero','BurnOutTime',[]);
names=fieldnames(provided);
for k=1:numel(names)
    name=names{k};
    assert(isfield(options,name),'RocketMotor:ImportOptionName', ...
        'Unsupported import option: %s.',name);
    value=provided.(name);
    if strcmp(name,'BurnOutTime')
        assert(isnumeric(value) && isreal(value) && ...
            (isempty(value) || (isscalar(value) && isfinite(value) && value>0)), ...
            'RocketMotor:BurnOutTime','BurnOutTime must be empty or a positive finite scalar in seconds.');
        options.(name)=double(value);
        continue
    end
    assert((ischar(value) && isrow(value) && ~isempty(value)) || ...
        (isstring(value) && isscalar(value) && ~ismissing(value) && strlength(value)>0), ...
        'RocketMotor:ImportOptionValue','%s must be a nonempty scalar string.',name);
    options.(name)=char(value);
end
assert(any(strcmp(options.EndpointMode,{'zero','abrupt'})), ...
    'RocketMotor:EndpointMode','EndpointMode must be zero or abrupt.');
reserved={'fuel_mass_kg','oxidizer_mass_kg','propellant_mass_kg'};
assert(~strcmp(options.TimeColumn,options.ThrustColumn) && ...
    ~any(ismember({options.TimeColumn,options.ThrustColumn},reserved)), ...
    'RocketMotor:ColumnMapping','Time/thrust columns must be distinct and cannot use mass-profile names.');
end

function code = parseType(motorType)
if isnumeric(motorType)
    assert(isscalar(motorType) && isreal(motorType) && isfinite(motorType) && ...
        any(motorType==[1,2,3]),'RocketMotor:MotorType', ...
        'Numeric motorType must be 1 solid, 2 hybrid, or 3 liquid.');
    code = double(motorType);
else
    assert((ischar(motorType) && isrow(motorType)) || ...
        (isstring(motorType) && isscalar(motorType)),'RocketMotor:MotorType', ...
        'motorType must be solid, hybrid, liquid, or code 1/2/3.');
    name = lower(char(motorType));
    if strcmp(name,'solid'), code=1;
    elseif strcmp(name,'hybrid'), code=2;
    elseif strcmp(name,'liquid'), code=3;
    else
        error('RocketMotor:MotorType','Unsupported motor type: %s.',name);
    end
end
end

function Properties = validateProperties(properties,typeCode)
required = {'DryMass','DryInertia','FuelInitialMass','FuelResidualMass', ...
    'OxidizerInitialMass','OxidizerResidualMass','FuelInitialInertia', ...
    'FuelResidualInertia','OxidizerInitialInertia','OxidizerResidualInertia'};
positions = {'DryCenterOfMassZ','FuelInitialCenterOfMassZ', ...
    'FuelResidualCenterOfMassZ','OxidizerInitialCenterOfMassZ', ...
    'OxidizerResidualCenterOfMassZ'};
grainFields = {'GrainInertiaEnabled','GrainCount','GrainDensity', ...
    'GrainOuterRadius','GrainInitialInnerRadius','GrainInitialHeight','GrainSeparation'};
assert(isstruct(properties) && isscalar(properties),'RocketMotor:PropertiesType', ...
    'properties must be a scalar structure.');
names = fieldnames(properties);
assert(all(ismember(required,names)),'RocketMotor:MissingProperties', ...
    'Missing motor properties: %s.',strjoin(required(~ismember(required,names)),', '));
allowed = [required,positions,grainFields];
assert(all(ismember(names,allowed)),'RocketMotor:UnknownProperties', ...
    'Unsupported motor properties: %s.',strjoin(names(~ismember(names,allowed)),', '));
Properties = struct();
for k=1:numel(required)
    name = required{k};
    value = properties.(name);
    assert(isnumeric(value) && isreal(value) && all(isfinite(value(:))) && ...
        all(value(:)>=0),'RocketMotor:PropertyValue', ...
        '%s must contain finite real nonnegative numeric values.',name);
    if contains(name,'Inertia')
        assert(isequal(size(value),[3,1]),'RocketMotor:InertiaShape', ...
            '%s must be a 3-by-1 inertia vector.',name);
    else
        assert(isscalar(value),'RocketMotor:MassShape','%s must be scalar.',name);
    end
    Properties.(name) = full(double(value));
end
for k=1:numel(positions)
    name = positions{k};
    value = 0;
    if isfield(properties,name)
        value = properties.(name);
    end
    assert(isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value), ...
        'RocketMotor:CenterOfMassPosition', ...
        '%s must be a finite real numeric axial position in meters.',name);
    Properties.(name) = full(double(value));
end
assert(Properties.FuelResidualMass<=Properties.FuelInitialMass && ...
    Properties.OxidizerResidualMass<=Properties.OxidizerInitialMass, ...
    'RocketMotor:ResidualMass','Residual mass cannot exceed initial mass.');
for k=1:numel(grainFields)
    name = grainFields{k};
    value = 0;
    if isfield(properties,name), value=properties.(name); end
    assert(isnumeric(value) && isreal(value) && isscalar(value) && ...
        isfinite(value) && value>=0,'RocketMotor:GrainProperty', ...
        '%s must be a finite real nonnegative numeric scalar.',name);
    Properties.(name) = full(double(value));
end
assert(any(Properties.GrainInertiaEnabled==[0,1]),'RocketMotor:GrainEnable', ...
    'GrainInertiaEnabled must be numeric zero or one.');
if Properties.GrainInertiaEnabled>0.5
    assert(typeCode==1,'RocketMotor:GrainMotorType', ...
        'BATES grain inertia is supported only for solid motors.');
    assert(Properties.GrainCount>=1 && Properties.GrainCount==floor(Properties.GrainCount) && ...
        Properties.GrainDensity>0 && Properties.GrainOuterRadius>Properties.GrainInitialInnerRadius && ...
        Properties.GrainInitialHeight>0,'RocketMotor:GrainGeometry', ...
        'BATES geometry needs a positive integer count, positive density/height and inner radius below outer radius.');
    geometryMass = Properties.GrainCount*Properties.GrainDensity*pi * ...
        (Properties.GrainOuterRadius^2-Properties.GrainInitialInnerRadius^2)*Properties.GrainInitialHeight;
    assert(isfinite(geometryMass) && geometryMass>0 && ...
        abs(geometryMass-Properties.FuelInitialMass)<=1e-10*geometryMass, ...
        'RocketMotor:GrainInitialMass','Initial propellant mass must match grain volume times density.');
    assert(Properties.FuelInitialCenterOfMassZ==Properties.FuelResidualCenterOfMassZ, ...
        'RocketMotor:GrainCenter','Symmetric BATES grains require a fixed propellant centroid.');
    % Geometry is authoritative. Repeating preparation derives identical
    % endpoints, including a partly burnt nonzero residual mass.
    Properties.FuelInitialInertia = rocketMotorSolidGrainInertia(Properties.FuelInitialMass,0,Properties);
    Properties.FuelResidualInertia = rocketMotorSolidGrainInertia(Properties.FuelResidualMass,0,Properties);
end
assert(Properties.DryMass>0 || all(Properties.DryInertia==0), ...
    'RocketMotor:MasslessInertia','Zero dry motor mass requires zero dry inertia.');
validateChannelInertia(Properties.FuelInitialMass,Properties.FuelResidualMass, ...
    Properties.FuelInitialInertia,Properties.FuelResidualInertia,'fuel');
validateChannelInertia(Properties.OxidizerInitialMass,Properties.OxidizerResidualMass, ...
    Properties.OxidizerInitialInertia,Properties.OxidizerResidualInertia,'oxidizer');
assert(Properties.FuelInitialMass~=Properties.FuelResidualMass || ...
    Properties.FuelInitialCenterOfMassZ==Properties.FuelResidualCenterOfMassZ, ...
    'RocketMotor:UnconsumedCenter', ...
    'Unconsumed fuel requires identical initial and residual center positions.');
assert(Properties.OxidizerInitialMass~=Properties.OxidizerResidualMass || ...
    Properties.OxidizerInitialCenterOfMassZ==Properties.OxidizerResidualCenterOfMassZ, ...
    'RocketMotor:UnconsumedCenter', ...
    'Unconsumed oxidizer requires identical initial and residual center positions.');
fuelConsumed = Properties.FuelInitialMass-Properties.FuelResidualMass;
oxidizerConsumed = Properties.OxidizerInitialMass-Properties.OxidizerResidualMass;
if typeCode==1
    assert(Properties.OxidizerInitialMass==0 && Properties.OxidizerResidualMass==0, ...
        'RocketMotor:SolidOxidizerChannel', ...
        'Solid motors use the fuel/propellant channel; the separate oxidizer mass must be zero.');
else
    assert(fuelConsumed>0 && oxidizerConsumed>0,'RocketMotor:ChannelConsumption', ...
        'Hybrid and liquid motors must consume both fuel and oxidizer.');
end
end

function validateChannelInertia(initialMass,residualMass,initialInertia,residualInertia,name)
assert((initialMass>0 || all(initialInertia==0)) && ...
    (residualMass>0 || all(residualInertia==0)), ...
    'RocketMotor:MasslessInertia','Zero %s mass requires zero corresponding inertia.',name);
assert(initialMass~=residualMass || isequal(initialInertia,residualInertia), ...
    'RocketMotor:UnconsumedInertia', ...
    'Unconsumed %s requires identical initial and residual inertia.',name);
end

function values = numericColumn(data,name)
values = data.(name);
assert(isnumeric(values) && isreal(values) && size(values,2)==1 && ...
    all(isfinite(values(:))),'RocketMotor:ColumnValue', ...
    'Column %s must be a finite real numeric scalar column.',name);
values = full(double(values));
end

function fraction = profileFraction(mass,initial,residual,name)
assert(all(mass>=residual) && all(mass<=initial) && all(diff(mass)<=0), ...
    'RocketMotor:MassProfile','%s mass profile must decrease monotonically within configured mass bounds.',name);
assert(mass(1)==initial && mass(end)==residual,'RocketMotor:MassEndpoints', ...
    '%s profile endpoints must equal configured initial and residual masses.',name);
if initial>residual
    fraction = (mass-residual)/(initial-residual);
else
    fraction = zeros(size(mass));
end
end

function description = massMethod(hasProfile,consumed)
if hasProfile
    description = 'Measured mass profile with piecewise-linear interpolation';
elseif consumed>0
    description = 'Impulse fallback; constant effective exhaust velocity';
else
    description = 'Unconsumed channel held at residual/initial mass';
end
end

function description = fuelInertiaMethod(enabled)
if enabled>0.5
    description = 'Symmetric BATES geometry reconstructed from actual remaining mass';
else
    description = 'Endpoint interpolation using remaining consumable fraction';
end
end
