%% Extract data from flight_data.csv

%exporter.export_data("flight_data.csv", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az",
%                     "e0", "e1", "e2", "e3", "w1", "w2", "w3", "latitude", "longitude", "altitude", "pressure",
%                      time_step = 0.02)
% File name
filename = "flight_data_corrected.csv";

% Read numeric data, skipping the header row
data = readmatrix(filename, "NumHeaderLines", 1);

% Extract columns
time_s  = data(:,1) + stationaryTime;

E       = data(:,2);
N       = data(:,3);
U       = data(:,4);

vE      = data(:,5);
vN      = data(:,6);
vU      = data(:,7);

aE      = data(:,8);
aN      = data(:,9);
aU      = data(:,10);

e0      = data(:,11);
e1      = data(:,12);
e2      = data(:,13);
e3      = data(:,14);

w1      = data(:,15);
w2      = data(:,16);
w3      = data(:,17);

lat_deg = data(:,18);
lon_deg = data(:,19);
press   = data(:,20);

p_NEU = [time_s, N, E, U];
v_NEU = [time_s, vN, vE, vU];
a_NEU = [time_s, aN, aE, aU];
% RocketPy attitude maps body to ENU. Apply the same ENU -> NEU -> NED
% reference conversion used by the trajectory vectors; keep body rates.
q = [time_s, enuToNedQuaternion([e1, e2, e3, e0])];
w = [time_s, w1, w2, w3];
pressure = [time_s, press];
LLA_rad = [time_s, deg2rad(lat_deg), deg2rad(lon_deg), U];

q0 = q(1,2:5)';   % Shared body-to-NED initial attitude, scalar-last.
%p0 = [N(1); E(1); -U(1)];
%v0 = [vN(1); vE(1); -vU(1)];
LLA0 = [deg2rad(lat_deg(1)); deg2rad(lon_deg(1)); 901.0];

