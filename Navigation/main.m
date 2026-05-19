clc
clearvars
close all

run SensorSettings.m

Environment.IGRF13.order = 3;
[Environment.IGRF13.g, Environment.IGRF13.h] = IGRF13;

q0 = [0;0;0;1];
p0 = [0;0;0];
v0 = [0;0;0];
ba0 = [0;0;0];
bg0 = [0;0;0];
P0(1:3,1:3)     = (5*pi/180)^2 * eye(3);
P0(4:6,4:6)     = 1.0^2 * eye(3);
P0(7:9,7:9)     = 10.0^2 * eye(3);
P0(10:12,10:12) = 0.01^2 * eye(3);
P0(13:15,13:15) = 0.1^2 * eye(3);
altitude_ASL_init = 0;