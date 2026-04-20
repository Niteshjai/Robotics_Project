function [nominal_params, true_params, configurations, measurements] = data_generator(...
    n_configs, perturbation_std, measurement_noise_std, seed)

if nargin < 1 || isempty(n_configs)
    n_configs = 100;
end

if nargin < 2 || isempty(perturbation_std)
    perturbation_std.a     = 0.0005;
    perturbation_std.alpha = deg2rad(0.02);
    perturbation_std.d     = 0.0005;
    perturbation_std.theta = deg2rad(0.02);
end

if nargin < 3 || isempty(measurement_noise_std), measurement_noise_std = 1e-4; end
if nargin < 4 || isempty(seed),                  seed = 42;                  end

nominal_params = [
     0.0000,  pi/2,   0.6718,  0;
     0.4318,  0,      0.0000,  0;
    -0.0203,  pi/2,   0.1500,  0;
     0.0000, -pi/2,   0.4318,  0;
     0.0000,  pi/2,   0.0000,  0;
     0.0000,  0,      0.0000,  0;
];

joint_limits = [
    -pi,    pi;
    -pi/2,  pi/2;
    -pi/2,  pi/2;
    -pi,    pi;
    -pi/2,  pi/2;
    -pi,    pi;
];

n_joints = size(nominal_params, 1);

rng(seed);
noise = [
    perturbation_std.a     * randn(n_joints, 1), ...
    perturbation_std.alpha * randn(n_joints, 1), ...
    perturbation_std.d     * randn(n_joints, 1), ...
    perturbation_std.theta * randn(n_joints, 1)
];
true_params = nominal_params + noise;

rng(seed + 1);
lo = joint_limits(:, 1)';
hi = joint_limits(:, 2)';
configurations = lo + (hi - lo) .* rand(n_configs, n_joints);

rng(seed + 2);
measurements = zeros(n_configs, 6);
for i = 1:n_configs
    q    = configurations(i, :)';
    T    = forward_kinematics(true_params, q);
    pose = pose_from_matrix(T);
    measurements(i, :) = pose' + measurement_noise_std * randn(1, 6);
end
end
