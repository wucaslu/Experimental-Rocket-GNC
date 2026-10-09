%% GNSS ellipsoid-height datum configuration
% RocketPy's documented elevation convention is above sea level. If this
% dataset was supplied with ellipsoid height instead, select zero correction.
% This launch-site geoid offset is shared by sensor output and Navigation's
% inverse conversion, so the local origin remains Down=0 in either case.
assert(isfinite(Sensor.GPS.SourceAltitudeIsMSL), ...
    'GNSS:SourceDatumRequired', 'Specify the source altitude datum first.');
if Sensor.GPS.SourceAltitudeIsMSL
    Sensor.GPS.GeoidSeparation = geoidheight( ...
        rad2deg(LLA0(1)), mod(rad2deg(LLA0(2)),360), 'EGM96');
else
    Sensor.GPS.GeoidSeparation = 0;
end
GNSSConfig = Sensor.GPS;

% Explicit receiver memory. Unit Delay resets these fields at simulation
% start; the MATLAB Function contains no hidden state or random stream.
GNSSInitialState = gnssPositionInitialize(LLA0, GNSSConfig);
GNSSReceiverStateBus = gnssPositionStateBus(GNSSInitialState);
