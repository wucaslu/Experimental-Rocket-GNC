%% Configurable demonstration rocket -- replace with measured rocket inputs.
% This original plant follows the structure of RocketPy's 6-DOF simulator,
% but uses a lumped, coincident-CM model with diagonal inertia. These values
% are NOT identified from flight_data.csv and are not a flight prediction.
% References:
% https://docs.rocketpy.org/en/latest/technical/equations_of_motion_v1.html
% https://docs.rocketpy.org/en/latest/user/rocket/rocket_usage.html
% https://docs.rocketpy.org/en/latest/reference/classes/Parachute.html
% SI units; body +Z points toward the nose; local navigation frame is NED.

Rocket = struct();
Rocket.Ts = 0.02;                % truth output / explicit state update [s]
Rocket.MaxStep = 0.002;          % RK4 internal integration step limit [s]
Rocket.IgnitionTime = 50;        % prelaunch sensor calibration interval [s]
Rocket.RailLength = 5;           % effective constrained travel [m]
Rocket.LaunchInclination = deg2rad(85); % above horizontal
Rocket.LaunchHeading = deg2rad(30);    % clockwise from North
Rocket.LaunchRoll = deg2rad(-135);     % about longitudinal body +Z

% Launch site and rail attitude come from the existing trajectory metadata.
% Runtime initialization does not read a CSV. Environment altitude is MSL;
% the existing GNSS datum conversion supplies ellipsoid height.
Rocket.Latitude = deg2rad(31.044839);
Rocket.Longitude = deg2rad(-103.536106);
Rocket.AltitudeMSL = 901;

Rocket.DryMass = 18;             % [kg]
Rocket.PropellantMass = 4.5;     % [kg]
Rocket.DryInertia = [9.4; 9.4; 0.05];        % [kg m^2], about common CM
Rocket.PropellantInertia = [0.3; 0.3; 0.007];% [kg m^2] at ignition
% Net installed motor thrust [N]; time relative to ignition [s]. Include
% zero-force endpoints. Propellant consumption is proportional to impulse.
Rocket.ThrustTime = [0; 0.05; 0.2; 1.5; 2.7; 3];
Rocket.ThrustForce = [0; 2800; 3400; 2800; 2300; 0];
Rocket.ThrustCumulativeImpulse = cumtrapz(Rocket.ThrustTime,Rocket.ThrustForce);
Rocket.ThrustCumulativeImpulse = Rocket.ThrustCumulativeImpulse / ...
    Rocket.ThrustCumulativeImpulse(end);
Rocket.ThrustDirection = [0; 0; 1];
Rocket.ThrustOffset = [0; 0; -0.6];          % relative to common CM [m]
% Diagonal nozzle gyration approximation [m^2] for angular exhaust flux.
Rocket.NozzleGyration = [0.25*(0.6^2+0.025^2); ...
                        0.25*(0.6^2+0.025^2); 0.5*0.025^2];

Rocket.ReferenceArea = pi*0.15^2/4;        % 150 mm diameter [m^2]
Rocket.ReferenceLength = 2.5;              % body length [m]
Rocket.AeroCP = [0; 0; -0.4];              % CP behind common CM [m]
Rocket.CNa = 8;                            % normal-force slope [1/rad]
Rocket.AngularDamping = [1; 1; 0.1];       % angular damping [N m s]
Rocket.AeroMach = [0; 0.7; 0.9; 1; 1.2; 2; 5];
Rocket.AeroCdPowered = [0.45; 0.45; 0.55; 0.75; 0.6; 0.5; 0.5];
Rocket.AeroCdCoast = [0.5; 0.5; 0.6; 0.8; 0.65; 0.55; 0.55];
Rocket.WindAltitude = [0; 5000; 15000];     % AGL [m]
Rocket.WindNED = [3 1 0; 6 2 0; 10 3 0];  % rows [North East Down], [m/s]

% Two additive recovery canopies, each with finite smooth inflation.
Rocket.DrogueLag = 1;                      % after apogee [s]
Rocket.DrogueCdS = 0.5;                    % drag coefficient * area [m^2]
Rocket.DrogueInflationTime = 0.8;           % [s]
Rocket.DrogueAttachment = [0; 0; 0.5];      % relative to CM [m]
Rocket.MainAltitudeAGL = 400;              % descending trigger [m]
Rocket.MainLag = 0.3;                      % after height crossing [s]
Rocket.MainCdS = 5.5;                      % [m^2], in addition to drogue
Rocket.MainInflationTime = 2;              % [s]
Rocket.MainAttachment = [0; 0; 0.5];
Rocket.IMUOffset = [0; 0; 0];              % accelerometer lever arm [m]

rocketPlantValidateConfig(Rocket);
