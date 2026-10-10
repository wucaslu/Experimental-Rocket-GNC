function [q,w,p_NEU,v_NEU,pressure] = rocketPlantKinematics(x,Rocket,Weather)
%ROCKETPLANTKINEMATICS Existing Environment truth interface.
% Quaternion is body->NED xyzw. Position is [North;East;absolute MSL Up],
% velocity [North;East;Up]. Ellipsoid conversion remains in GNSS model.
if nargin<3, Weather=[]; end
q=x(7:10)/max(norm(x(7:10)),eps);
w=x(11:13);
p_NEU=[x(1);x(2);Rocket.AltitudeMSL-x(3)];
v_NEU=[x(4);x(5);-x(6)];
[~,~,pressure]=rocketWeatherConditions(p_NEU(3),Rocket,Weather,zeros(3,1));
end
