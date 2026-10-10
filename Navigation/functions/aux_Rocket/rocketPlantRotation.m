function R = rocketPlantRotation(q)
%ROCKETPLANTROTATION Body to local NED DCM from Hamilton [x;y;z;w].
q = q/max(norm(q),eps);
v = q(1:3);
s = q(4);
K = [0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
R = (s*s-v.'*v)*eye(3)+2*(v*v.')+2*s*K;
end
