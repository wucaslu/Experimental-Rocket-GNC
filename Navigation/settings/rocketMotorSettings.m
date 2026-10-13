%% Static-fire motor interface with RocketPy Calisto/M1670 default parameters.
% The CSV is demonstration data, not an identified or certified motor curve.
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

% Both presets use the same stateless runtime motor interface. Geometry and
% numeric grain geometry enters runtime Motor; source metadata stays outside it.
if ~exist('RocketParameterSet','var')
    RocketParameterSet = 'calisto';
end
assert((ischar(RocketParameterSet) && isrow(RocketParameterSet)) || ...
    (isstring(RocketParameterSet) && isscalar(RocketParameterSet)), ...
    'RocketPlant:ParameterSet','RocketParameterSet must be calisto or generic.');
RocketParameterSet = lower(char(RocketParameterSet));
assert(any(strcmp(RocketParameterSet,{'calisto','generic'})), ...
    'RocketPlant:ParameterSet','RocketParameterSet must be calisto or generic.');
if strcmp(RocketParameterSet,'calisto')
    [CalistoRocketParameters,MotorProperties,MotorStaticFireSource,RocketPyCalistoInfo] = ...
        rocketPyCalistoParameters();
end

% Optional explicit initialization inputs. A path, table or numeric [t,T]
% array can be supplied; no files or strings are read by the runtime model.
if exist('RocketMotorStaticFireData','var')
    MotorStaticFireSource = RocketMotorStaticFireData;
end
if exist('RocketMotorType','var')
    MotorType = RocketMotorType;
end
if exist('RocketMotorProperties','var')
    MotorProperties = RocketMotorProperties;
end
[MotorStaticFire,MotorTypeCode,MotorProperties,MotorSetupInfo] = ...
    rocketMotorPrepare(MotorStaticFireSource,MotorType,MotorProperties);
Motor = struct('StaticFire',MotorStaticFire,'Type',MotorTypeCode, ...
    'Properties',MotorProperties);
