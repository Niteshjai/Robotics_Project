function [nominal_params, true_params, configurations, measurements] = data_generator(...
    n_configs, perturbation_std, measurement_noise_std, seed)


% --- Defaults ----------------------------------------------------------
if nargin < 1 || isempty(n_configs),             n_configs = 100;            end
if nargin < 2 || isempty(perturbation_std)
    perturbation_std.a     = 0.0005;           % 0.5 mm
    perturbation_std.alpha = deg2rad(0.02);    % 0.02 deg
    perturbation_std.d     = 0.0005;           % 0.5 mm
    perturbation_std.theta = deg2rad(0.02);    % 0.02 deg
end
if nargin < 3 || isempty(measurement_noise_std), measurement_noise_std = 1e-4; end
if nargin < 4 || isempty(seed),                  seed = 42;                  end

% --- Nominal DH parameters (Puma-560 inspired) -------------------------
% Columns: [a(m), alpha(rad), d(m), theta_offset(rad)]
nominal_params = [
     0.0000,  pi/2,   0.6718,  0;   % joint 1
     0.4318,  0,      0.0000,  0;   % joint 2
    -0.0203,  pi/2,   0.1500,  0;   % joint 3
     0.0000, -pi/2,   0.4318,  0;   % joint 4
     0.0000,  pi/2,   0.0000,  0;   % joint 5
     0.0000,  0,      0.0000,  0;   % joint 6
];

% Joint limits [min, max] rad  (approximate Puma-560)
joint_limits = [
    -pi,    pi;
    -pi/2,  pi/2;
    -pi/2,  pi/2;
    -pi,    pi;
    -pi/2,  pi/2;
    -pi,    pi;
];

n_joints = size(nominal_params, 1);

% --- Step 1: Perturb DH parameters -------------------------------------
rng(seed);
noise = [
    perturbation_std.a     * randn(n_joints, 1), ...
    perturbation_std.alpha * randn(n_joints, 1), ...
    perturbation_std.d     * randn(n_joints, 1), ...
    perturbation_std.theta * randn(n_joints, 1)
];
true_params = nominal_params + noise;

% --- Step 2: Random joint configurations --------------------------------
rng(seed + 1);
lo = joint_limits(:, 1)';
hi = joint_limits(:, 2)';
configurations = lo + (hi - lo) .* rand(n_configs, n_joints);

% --- Step 3: Simulate noisy pose measurements ---------------------------
rng(seed + 2);
measurements = zeros(n_configs, 6);
for i = 1:n_configs
    q    = configurations(i, :)';
    T    = forward_kinematics(true_params, q);
    pose = pose_from_matrix(T);
    measurements(i, :) = pose' + measurement_noise_std * randn(1, 6);
end
end
