function rpy = rotation_to_rpy(R)

pitch = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));

if abs(cos(pitch)) < 1e-10
    roll = 0;
    if pitch > 0
        yaw = atan2(-R(2,3), R(2,2));
    else
        yaw = atan2( R(2,3), -R(2,2));
    end
else
    roll = atan2(R(3,2), R(3,3));
    yaw  = atan2(R(2,1), R(1,1));
end

rpy = [roll; pitch; yaw];
end
