function rpy = rotation_to_rpy(R)
% ROTATION_TO_RPY  Extract Roll-Pitch-Yaw (XYZ extrinsic) from 3x3 rotation.
%
% Input:
%   R   - 3x3 rotation matrix
%
% Output:
%   rpy - [roll, pitch, yaw] in radians

pitch = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));

if abs(cos(pitch)) < 1e-10   % gimbal lock
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
