function x = rocketPlantInitialize(Rocket)
%ROCKETPLANTINITIALIZE Explicit initial state for the rocket truth plant.
% x = [p_NED(3);v_NED(3);q_BtoNED_xyzw(4);omega_B(3);phase;
%      apogeeTime;drogueDeploymentTime;mainDeploymentTime].
% Body +z points toward the nose. Inclination is above the horizontal;
% heading is clockwise from North and roll is about body +z, all radians.

inc = Rocket.LaunchInclination;
head = Rocket.LaunchHeading;
roll = Rocket.LaunchRoll;
b3 = [cos(inc)*cos(head); cos(inc)*sin(head); -sin(inc)];
b1 = [sin(inc)*cos(head); sin(inc)*sin(head); cos(inc)];
b2 = cross(b3,b1);
R = [cos(roll)*b1 + sin(roll)*b2, ...
    -sin(roll)*b1 + cos(roll)*b2, b3];
q = rocketPlantQuaternionFromDCM(R);
x = zeros(17,1);
x(7:10) = q;
x(15:17) = -1;
end
