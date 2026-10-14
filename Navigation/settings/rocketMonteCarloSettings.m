function options = rocketMonteCarloSettings()
%ROCKETMONTECARLOSETTINGS Editable example uncertainties for coupled flights.
% OPTIONS = ROCKETMONTECARLOSETTINGS() returns runner options and uncertainty
% scales. These illustrative values are not measured manufacturing/weather
% distributions. Replace them with static-fire, survey, calibration and flight
% evidence before interpreting probabilities as operational predictions.
% Positive multipliers use lognormal draws bounded at +/-TruncationSigma in
% their underlying normal variable. Additive errors use bounded zero-mean
% Gaussian draws. Angles are radians, lengths metres, time seconds, SI otherwise.
options.Seed=20261014;
options.Model='NavigationRocketPlant';
options.ParameterSet='irec';
options.Scenario='irec-baseline';
options.WeatherSource='rocketpy-airbrakes';
options.CdLookupSource='irec-nominal';
options.FlightDurationS=350; % after nominal ignition; short runs censor landing
options.OutputDirectory=''; % unique folder below RocketPlant/validation/monte-carlo
options.RunSimulations=true;
options.SaveSimulations=false; % full SimulationOutput is large; manifest always saved
options.MakePlots=true;
options.RandomizeNoiseSeeds=true; % existing noise levels/calibration stay fixed
options.TargetToleranceM=10;
options.ControlEnabled=[]; % [] retains nominal initialization setting
options.TargetApogeeAGL=[];
options.TruncationSigma=3;
options.NominalVariables=[]; % optional complete initialized variables snapshot
options.MotorStaticFireData=[]; % optional measured file/table/time-thrust matrix
options.MotorDataset='';       % optional guarded dataset, e.g. mandioca-2024
options.MotorImportOptions=struct(); % explicit column mapping/endpoint policy
options.MotorType=[];
options.MotorProperties=[];

u.DryMassRelative=.02;
u.DryInertiaRelative=.05;
u.AirframeCenterOfMassM=.005;
u.BodyRadiusRelative=.003;
u.FinDimensionsRelative=.005;
u.FinCantRad=deg2rad(.05);
u.FinLiftSlopeRelative=.03;
u.DragAreaRelative=.05; % effective axial Cd*A uncertainty, not a radius error
u.MotorDryMassRelative=.02;
u.MotorInertiaRelative=.05;
u.MotorPositionM=.003;
u.MotorDryCenterOfMassM=.003;
u.GrainDensityRelative=.01;
u.GrainDimensionsRelative=.003;
u.FuelMassRelative=.02; % only non-geometric motors; BATES mass follows geometry
u.OxidizerMassRelative=.02; % inactive for a solid without a separate oxidizer
u.ExhaustVelocityRelative=.02;
u.BurnTimeRelative=.02;
u.RailLengthRelative=.01;
u.LaunchInclinationRad=deg2rad(.25);
u.LaunchHeadingRad=deg2rad(1);
u.LaunchRollRad=deg2rad(1);
u.LaunchAltitudeM=0; % optional surveyed-site variation; datum rebuilt when changed
u.IgnitionTimeS=0; % onboard stationary interval stays nominal if this is varied
u.CanopyCdSRelative=.10;
u.CanopyInflationRelative=.10;
u.RecoveryLagS=.10;
u.MainTriggerAltitudeM=10;
u.TemperatureK=3;
u.PressureRelative=.01;
u.LapseRateKpm=.0002;
u.RelativeHumidity=.10;
u.WindOffsetNED=[2;2;.25]; % additive: a zero nominal wind still varies
u.WindShearNED=[.4;.4;.05]; % m/s per km AGL
u.AdditionalGustRmsNED=[.6;.6;.1]; % magnitude of bounded normal draws
u.NavPositionNEDM=[.5;.5;1];
u.NavVelocityNEDMps=[.1;.1;.1];
u.NavAttitudeRad=deg2rad([1;1;2]);
u.NavGyroBiasRadps=.005*ones(3,1);
u.NavAccelBiasMps2=.05*ones(3,1);
options.Uncertainty=u;
end
