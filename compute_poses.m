function poses = compute_poses(dh_params, configurations)

m = size(configurations, 1);
poses = zeros(m, 6);
for i = 1:m
    T        = forward_kinematics(dh_params, configurations(i, :)');
    poses(i,:) = pose_from_matrix(T)';
end
end
