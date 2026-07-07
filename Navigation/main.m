clc
close all

addpath('functions/')
addpath('functions/aux_math/')
addpath('functions/aux_MEKF/')
addpath('settings/')
addpath('settings/environment/')

Environment.IGRF13.order = 3;
[Environment.IGRF13.g, Environment.IGRF13.h] = IGRF13;

stationaryTime = 50; 
marginTime = 5;
run csv_extract.m

P0 = zeros(15, 15);
g0 = 9.80665;   % gravity [m/s^2]

p0 = [0; 0; 0];
v0 = [0;0;0];
ba0 = [1;1;1]*0;
bg0 = [1;1;1]*0;
P0(1:3,1:3)     = (5*pi/180)^2 * eye(3);
P0(4:6,4:6)     = 5.0^2 * eye(3);
P0(7:9,7:9)     = 5.0^2 * eye(3);
P0(10:12,10:12) = 0.1^2 * eye(3);
P0(13:15,13:15) = 3^2 * eye(3);
[pressure0, ~] = ISA_model(0, LLA0(3));
