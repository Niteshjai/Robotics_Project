function pose = pose_from_matrix(T)
% POSE_FROM_MATRIX  Extract 6-DOF pose vector from 4x4 homogeneous transform.
%
% Input:
%   T    - 4x4 homogeneous transform
%
% Output:
%   pose - 6x1 vector [x; y; z; roll; pitch; yaw]

xyz  = T(1:3, 4);
rpy  = rotation_to_rpy(T(1:3, 1:3));
pose = [xyz; rpy];
end
