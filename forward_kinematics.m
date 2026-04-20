function T = forward_kinematics(dh_params, joint_angles)

n = size(dh_params, 1);
assert(n == length(joint_angles), ...
    'Number of DH rows must match number of joint angles.');

T = eye(4);
for i = 1:n
    a     = dh_params(i, 1);
    alpha = dh_params(i, 2);
    d     = dh_params(i, 3);
    theta = dh_params(i, 4) + joint_angles(i);
    T     = T * dh_transform(a, alpha, d, theta);
end
end
