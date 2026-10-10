function q = rocketPlantQuaternionFromDCM(R)
%ROCKETPLANTQUATERNIONFROMDCM Active Hamilton quaternion, scalar last.
tr = R(1,1)+R(2,2)+R(3,3);
q = zeros(4,1);
if tr > 0
    s = 2*sqrt(max(0,1+tr));
    q(4) = 0.25*s;
    q(1) = (R(3,2)-R(2,3))/s;
    q(2) = (R(1,3)-R(3,1))/s;
    q(3) = (R(2,1)-R(1,2))/s;
elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
    s = 2*sqrt(max(0,1+R(1,1)-R(2,2)-R(3,3)));
    q(1) = 0.25*s;
    q(2) = (R(1,2)+R(2,1))/s;
    q(3) = (R(1,3)+R(3,1))/s;
    q(4) = (R(3,2)-R(2,3))/s;
elseif R(2,2) > R(3,3)
    s = 2*sqrt(max(0,1-R(1,1)+R(2,2)-R(3,3)));
    q(1) = (R(1,2)+R(2,1))/s;
    q(2) = 0.25*s;
    q(3) = (R(2,3)+R(3,2))/s;
    q(4) = (R(1,3)-R(3,1))/s;
else
    s = 2*sqrt(max(0,1-R(1,1)-R(2,2)+R(3,3)));
    q(1) = (R(1,3)+R(3,1))/s;
    q(2) = (R(2,3)+R(3,2))/s;
    q(3) = 0.25*s;
    q(4) = (R(2,1)-R(1,2))/s;
end
q = q/max(norm(q),eps);
if q(4)<0
    q=-q;
end
end
