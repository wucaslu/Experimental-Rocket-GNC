%% Static-fire motor interface with selectable IREC/Mandioca and examples.
% Preset curves are examples. RocketMotorDataset='mandioca-2024' selects the
% supplied SI static-fire record with an explicit abrupt endpoint policy;
% this selection requires a complete supplied RocketMotorProperties struct.
% Select 'solid', 'hybrid', or 'liquid'. Hybrid/liquid configurations require
% separate initial/residual fuel and oxidizer masses. Optional CSV columns
% fuel_mass_kg and oxidizer_mass_kg supply independently measured histories;
% propellant_mass_kg is also accepted for solid motors.
% Diagonal inertias are intrinsic about each component's own center of mass.
% All axial center positions use the fixed body reference O (+Z noseward).
% Dry motor hardware belongs here; Rocket.DryMass excludes this contribution.
% The generic 2 kg motor/16 kg airframe partition preserves the previous
% total 18 kg burnout mass and is an editable demonstration assumption.
motorSettingsDirectory = fileparts(mfilename('fullpath'));
MotorStaticFireSource = fullfile(motorSettingsDirectory,'motor_static_fire_example.csv');
MotorType = 'solid';
MotorProperties = struct();
MotorProperties.DryMass = 2.0;                 % retained motor hardware [kg]
MotorProperties.DryCenterOfMassZ = -0.2;       % generic motor hardware CM [m]
MotorProperties.DryInertia = [0.42;0.42;0.005];% intrinsic [kg m^2]
MotorProperties.FuelInitialMass = 4.5;         % solid propellant or fuel [kg]
MotorProperties.FuelResidualMass = 0.0;        % retained after burn [kg]
MotorProperties.OxidizerInitialMass = 0.0;     % separate oxidizer [kg]
MotorProperties.OxidizerResidualMass = 0.0;    % retained after burn [kg]
% Symmetric solid-grain regression keeps its own centroid fixed; changing
% propellant mass shifts the total rocket CM from -0.04 m at ignition to 0 m.
MotorProperties.FuelInitialCenterOfMassZ = -0.2;
MotorProperties.FuelResidualCenterOfMassZ = -0.2;
MotorProperties.OxidizerInitialCenterOfMassZ = 0;
MotorProperties.OxidizerResidualCenterOfMassZ = 0;
MotorProperties.FuelInitialInertia = [0.12;0.12;0.007];
MotorProperties.FuelResidualInertia = zeros(3,1);
MotorProperties.OxidizerInitialInertia = zeros(3,1);
MotorProperties.OxidizerResidualInertia = zeros(3,1);

% All presets use the same stateless runtime motor interface. Geometry and
% numeric grain geometry enters runtime Motor; source metadata stays outside it.
if ~exist('RocketParameterSet','var')
    RocketParameterSet = 'calisto';
end
assert((ischar(RocketParameterSet) && isrow(RocketParameterSet)) || ...
    (isstring(RocketParameterSet) && isscalar(RocketParameterSet)), ...
    'RocketPlant:ParameterSet','RocketParameterSet must be irec, calisto or generic.');
RocketParameterSet = lower(char(RocketParameterSet));
assert(any(strcmp(RocketParameterSet,{'irec','calisto','generic'})), ...
    'RocketPlant:ParameterSet','RocketParameterSet must be irec, calisto or generic.');
if strcmp(RocketParameterSet,'calisto')
    [CalistoRocketParameters,MotorProperties,MotorStaticFireSource,RocketPyCalistoInfo] = ...
        rocketPyCalistoParameters();
end
irecMotorImportOptions=struct();
if strcmp(RocketParameterSet,'irec')
    irecAssetDirectory=fullfile(motorSettingsDirectory,'mandioca_2024');
    irecDragSource=fullfile(irecAssetDirectory,'CD_Superatualizado.csv');
    irecAirfoilSource=fullfile(irecAssetDirectory,'NACA0012-radians.csv');
    irecAirfoilAngleUnit='radians';
    irecPhysicalRailLength=5.2;
    irecBurnOutTime=3.99;
    if exist('RocketIrecDragData','var'),irecDragSource=RocketIrecDragData;end
    if exist('RocketIrecAirfoilData','var'),irecAirfoilSource=RocketIrecAirfoilData;end
    if exist('RocketIrecAirfoilAngleUnit','var'),irecAirfoilAngleUnit=RocketIrecAirfoilAngleUnit;end
    if exist('RocketPhysicalRailLength','var'),irecPhysicalRailLength=RocketPhysicalRailLength;end
    if exist('RocketMotorBurnOutTime','var'),irecBurnOutTime=RocketMotorBurnOutTime;end
    [IrecRocketParameters,MotorProperties,MotorStaticFireSource,IrecSetupInfo,irecMotorImportOptions]= ...
        rocketIrecParameters(irecDragSource,irecAirfoilSource, ...
        irecPhysicalRailLength,irecBurnOutTime,irecAirfoilAngleUnit);
end

% Optional explicit initialization inputs. A path, table or numeric [t,T]
% array can be supplied; no files or strings are read by the runtime model.
if exist('RocketMotorStaticFireData','var')
    MotorStaticFireSource = RocketMotorStaticFireData;
    % A newly supplied source has its own schema and burn interval. Never
    % apply the IREC file's column mapping/cutoff to an unrelated dataset.
    irecMotorImportOptions=struct();
end
if exist('RocketMotorType','var')
    MotorType = RocketMotorType;
end
if exist('RocketMotorProperties','var')
    MotorProperties = RocketMotorProperties;
end
MotorImportOptions=irecMotorImportOptions;
if exist('RocketMotorImportOptions','var')
    MotorImportOptions=RocketMotorImportOptions;
end
MotorDataset='preset';
if exist('RocketMotorDataset','var')
    assert((ischar(RocketMotorDataset) && isrow(RocketMotorDataset)) || ...
        (isstring(RocketMotorDataset) && isscalar(RocketMotorDataset)), ...
        'RocketMotor:Dataset','RocketMotorDataset must be preset or mandioca-2024.');
    MotorDataset=lower(char(RocketMotorDataset));
    assert(any(strcmp(MotorDataset,{'preset','mandioca-2024'})), ...
        'RocketMotor:Dataset','RocketMotorDataset must be preset or mandioca-2024.');
end
if strcmp(MotorDataset,'mandioca-2024')
    assert(strcmp(RocketParameterSet,'irec') || ...
        (exist('RocketMotorProperties','var')==1 && ...
        isstruct(RocketMotorProperties) && isscalar(RocketMotorProperties)), ...
        'RocketMotor:MandiocaPropertiesRequired', ...
        'Supply measured RocketMotorProperties before selecting Mandioca; preset motor mass/geometry cannot be substituted.');
    assert(exist('RocketMotorStaticFireData','var')~=1 && ...
        exist('RocketMotorImportOptions','var')~=1, ...
        'RocketMotor:DatasetConflict', ...
        'Select Mandioca via RocketMotorDataset or explicit data/import options, not both.');
    MotorStaticFireSource=fullfile(motorSettingsDirectory,'mandioca_2024','mandioca_2024.csv');
    if strcmp(RocketParameterSet,'irec')
        MotorImportOptions=irecMotorImportOptions;
    else
        MotorImportOptions=struct('TimeColumn','burn_time', ...
            'ThrustColumn','thrust_corrected','EndpointMode','abrupt');
    end
end
[MotorStaticFire,MotorTypeCode,MotorProperties,MotorSetupInfo] = ...
    rocketMotorPrepare(MotorStaticFireSource,MotorType,MotorProperties,MotorImportOptions);
if strcmp(MotorDataset,'mandioca-2024')
    assert(MotorTypeCode==1,'RocketMotor:MandiocaType', ...
        'The Mandioca dataset is configured for a solid motor.');
end
MotorSetupInfo.Dataset=MotorDataset;
Motor = struct('StaticFire',MotorStaticFire,'Type',MotorTypeCode, ...
    'Properties',MotorProperties);
