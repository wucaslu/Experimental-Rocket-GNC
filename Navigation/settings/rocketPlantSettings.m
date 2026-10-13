%% RocketPy Calisto example, or the retained generic demonstration preset.
% This original plant follows the structure of RocketPy's 6-DOF simulator,
% with axial time-varying center of mass and diagonal inertia. These values
% are NOT identified from flight_data.csv and are not a flight prediction.
% References:
% https://docs.rocketpy.org/en/latest/technical/equations_of_motion_v1.html
% https://docs.rocketpy.org/en/latest/user/rocket/rocket_usage.html
% https://docs.rocketpy.org/en/latest/reference/classes/Parachute.html
% https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/fins/fins.html
% SI units; body +Z points toward the nose; local navigation frame is NED.
% RocketParameterSet='calisto' is the default; select 'generic' explicitly to
% reproduce the previous demonstration. Notebook values are example inputs.

Rocket = struct();
Rocket.Ts = 0.02;                % truth output / explicit state update [s]
Rocket.MaxStep = 0.002;          % RK4 internal integration step limit [s]
Rocket.IgnitionTime = 50;        % prelaunch sensor calibration interval [s]
Rocket.MomentumModel = 0;       % 0: physical derivation; 1: RocketPy 1.13 code
Rocket.IncludeCoriolis = 0;     % local Earth-rotation acceleration
Rocket.RailLength = 5;           % effective constrained travel [m]
Rocket.LaunchInclination = deg2rad(85); % above horizontal
Rocket.LaunchHeading = deg2rad(30);    % clockwise from North
Rocket.LaunchRoll = deg2rad(-135);     % about longitudinal body +Z

% Launch site and rail attitude come from the existing trajectory metadata.
% Runtime initialization does not read a trajectory CSV. Environment altitude
% is MSL; the existing GNSS datum conversion supplies ellipsoid height.
Rocket.Latitude = deg2rad(31.044839);
Rocket.Longitude = deg2rad(-103.536106);
Rocket.AltitudeMSL = 901;

Rocket.DryMass = 16;             % airframe/payload without motor [kg]
Rocket.DryCenterOfMassZ = 0.025;  % generic airframe CM in fixed body frame [m]
Rocket.DryInertia = [8.89; 8.89; 0.045]; % intrinsic airframe inertia [kg m^2]
% Separate motor preparation imports static-fire data once at initialization.
% The generic partition preserves the previous total dry mass and inertia.
run(fullfile(fileparts(mfilename('fullpath')),'rocketMotorSettings.m'));
Rocket.Motor = Motor;
% Installation geometry remains a rocket property, relative to its reference.
Rocket.ThrustDirection = [0; 0; 1];
Rocket.ThrustOffset = [0; 0; -0.6];          % fixed body nozzle station [m]
% Diagonal nozzle gyration approximation [m^2] for angular exhaust flux.
Rocket.NozzleGyration = [0.25*(0.6^2+0.025^2); ...
                        0.25*(0.6^2+0.025^2); 0.5*0.025^2];

% Demonstration geometry. Z positions are body coordinates relative to the
% body reference O; positive Z is noseward. Match these to your actual rocket.
% O is the combined dry CM for the generic configuration. It never moves
% relative to the airframe; the instantaneous total CM is calculated at runtime.
Rocket.BodyRadius = 0.075;                 % [m]
Rocket.ReferenceArea = pi*Rocket.BodyRadius^2;
Rocket.ReferenceLength = 2.5;              % metadata: overall length [m]
Rocket.AeroEnabled = 1;
Rocket.UseCdLookup = 1;         % 1: velocity/actuator table; 0: Mach curves
Rocket.CdLookupIsIncremental = 0; % 1: airbrake delta Cd added to nominal curve
Rocket.NoseLength = 0.5;
Rocket.NoseBaseRadius = Rocket.BodyRadius;
Rocket.NoseTipZ = 1.25;
Rocket.NoseCPFraction = 2/3;                % conical; CP aft of tip / length
Rocket.FinNumber = 4;
Rocket.FinRootChord = 0.38;
Rocket.FinTipChord = 0.14;
Rocket.FinSpan = 0.15;                      % exposed span beyond body [m]
Rocket.FinSweepLength = 0.14;               % tip leading edge aft of root [m]
Rocket.FinLeadingEdgeZ = -0.65;
Rocket.FinCant = 0;                        % [rad], positive drives +Z roll
Rocket.FinLiftSlope2D = 2*pi;              % flat-plate default [1/rad]
Rocket.TailLength = 0.15;
Rocket.TailTopRadius = Rocket.BodyRadius;
Rocket.TailBottomRadius = 0.06;
Rocket.TailTopZ = -1.1;
Rocket.AngularDamping = zeros(3,1);         % optional residual [N m s]
Rocket.AeroMach = [0; 0.7; 0.9; 1; 1.2; 2; 5];
Rocket.AeroCdPowered = [0.45; 0.45; 0.55; 0.75; 0.6; 0.5; 0.5];
Rocket.AeroCdCoast = [0.5; 0.5; 0.6; 0.8; 0.65; 0.55; 0.55];
Rocket.WindAltitude = [0; 5000; 15000];     % AGL [m]
Rocket.WindNED = [3 1 0; 6 2 0; 10 3 0];  % rows [North East Down], [m/s]

% Two additive recovery canopies, each with finite smooth inflation.
Rocket.DrogueLag = 1;                      % after apogee [s]
Rocket.DrogueCdS = 0.5;                    % drag coefficient * area [m^2]
Rocket.DrogueInflationTime = 0.8;           % [s]
Rocket.DrogueAttachment = [0; 0; 0.5];      % fixed body station [m]
Rocket.MainAltitudeAGL = 400;              % descending trigger [m]
Rocket.MainLag = 0.3;                      % after height crossing [s]
Rocket.MainCdS = 5.5;                      % [m^2], in addition to drogue
Rocket.MainInflationTime = 2;              % [s]
Rocket.MainAttachment = [0; 0; 0.5];
Rocket.IMUOffset = [0; 0; 0];              % fixed IMU station from body O [m]

% Apply the selected example's geometry and recovery properties. The motor
% settings also prepare its thrust data and translate component centroids.
% Calisto also supplies the executed notebook's launch site, rail travel and
% initial body attitude after ENU->NED conversion; weather is selected later.
if strcmp(RocketParameterSet,'calisto')
    calistoNames = fieldnames(CalistoRocketParameters);
    for calistoIndex=1:numel(calistoNames)
        Rocket.(calistoNames{calistoIndex}) = CalistoRocketParameters.(calistoNames{calistoIndex});
    end
end
% Initialization-only scenario; runtime Rocket contains numeric properties.
% Direct settings users retain the original notebook configuration unless
% this scenario is selected explicitly by rocketPlantMain or the caller.
if strcmp(RocketParameterSet,'calisto') && exist('RocketScenario','var') && ...
        strcmp(RocketScenario,'rocketpy-airbrakes')
    Rocket.AltitudeMSL=1400;
    Rocket.FinCant=deg2rad(0.5);
    Rocket.UseCdLookup=1;
    Rocket.CdLookupIsIncremental=1;
end
Rocket = rocketPlantBuildAero(Rocket);
rocketPlantValidateConfig(Rocket);
