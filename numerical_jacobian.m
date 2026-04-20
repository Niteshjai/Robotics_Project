function J = numerical_jacobian(dh_params, joint_angles, delta)

if nargin < 3
    delta = 1e-7;
end

n_joints = size(dh_params, 1);
n_params  = n_joints * 4;
J = zeros(6, n_params);

T0     = forward_kinematics(dh_params, joint_angles);
pose0  = pose_from_matrix(T0);

params_flat = reshape(dh_params', [], 1);

for k = 1:n_params
    p_pert    = params_flat;
    p_pert(k) = p_pert(k) + delta;
    dh_pert   = reshape(p_pert, 4, n_joints)';
    T_pert    = forward_kinematics(dh_pert, joint_angles);
    pose_pert = pose_from_matrix(T_pert);
    J(:, k)   = (pose_pert - pose0) / delta;
end
end
