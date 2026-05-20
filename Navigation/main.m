clc
close all

Environment.IGRF13.order = 3;
[Environment.IGRF13.g, Environment.IGRF13.h] = IGRF13;

q0 = [0;0;0;1];
p0 = [0;0;0];
v0 = [0;0;0];
ba0 = [1;1;1]*0;
bg0 = [1;1;1]*0;
P0(1:3,1:3)     = (5*pi/180)^2 * eye(3);
P0(4:6,4:6)     = 1.0^2 * eye(3);
P0(7:9,7:9)     = 10.0^2 * eye(3);
P0(10:12,10:12) = 0.01^2 * eye(3);
P0(13:15,13:15) = 1^2 * eye(3);
pressure0 = 101325;
stationaryTime = 50; 