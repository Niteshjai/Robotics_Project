
clear; clc; close all;

% Get current directory
current_dir = fileparts(mfilename('fullpath'));
cd(current_dir);

% -------------------------------------------------------------------------
% Configuration
% -------------------------------------------------------------------------

RESULTS_DIR   = fullfile(current_dir, 'results');
N_CONFIGS     = 100;         % total configurations
TEST_FRACTION = 0.20;        % fraction held out for testing
SEED          = 42;

% DH perturbation sizes (1-sigma)
PERTURBATION_STD.a     = 0.0005;           % 0.5 mm
PERTURBATION_STD.alpha = deg2rad(0.02);    % 0.02 deg
PERTURBATION_STD.d     = 0.0005;           % 0.5 mm
PERTURBATION_STD.theta = deg2rad(0.02);    % 0.02 deg

MEASUREMENT_NOISE_STD = 1e-4;   % 0.1 mm / ~0.006 deg

% -------------------------------------------------------------------------
% Step 1 – Generating Calibration Dataset
% -------------------------------------------------------------------------
fprintf('\n────────────────────────────────────────────────────────────────────────\n');
fprintf('  Step 1 – Generating Calibration Dataset\n');
fprintf('────────────────────────────────────────────────────────────────────────\n');

[nominal, true_params, configs, measurements] = data_generator(...
    N_CONFIGS, PERTURBATION_STD, MEASUREMENT_NOISE_STD, SEED);

n_joints = size(nominal, 1);
fprintf('  Robot            : %d-DOF (Puma-560 inspired)\n', n_joints);
fprintf('  Configurations   : %d  (training + test)\n', N_CONFIGS);
fprintf('  Measurement noise: %.2f mm / %.4f mdeg\n', ...
    MEASUREMENT_NOISE_STD * 1000, MEASUREMENT_NOISE_STD * 1000 * 180 / pi);

% -------------------------------------------------------------------------
% Step 2 – Train / test split
% -------------------------------------------------------------------------
n_test  = max(1, round(N_CONFIGS * TEST_FRACTION));
n_train = N_CONFIGS - n_test;

rng(SEED + 99);
idx = randperm(N_CONFIGS);
train_idx = idx(1:n_train);
test_idx  = idx(n_train+1:end);

train_configs = configs(train_idx, :);
train_meas    = measurements(train_idx, :);
test_configs  = configs(test_idx, :);
test_meas     = measurements(test_idx, :);

fprintf('\n  Training configs : %d\n', n_train);
fprintf('  Test configs     : %d\n', n_test);

% -------------------------------------------------------------------------
% Step 3 – True (Injected) DH Parameter Perturbations
% -------------------------------------------------------------------------
fprintf('\n────────────────────────────────────────────────────────────────────────\n');
fprintf('  Step 2 – True (Injected) DH Parameter Perturbations\n');
fprintf('────────────────────────────────────────────────────────────────────────\n');

delta_true = true_params - nominal;
labels = {'a', '\alpha', 'd', '\theta'};
col_units = {'mm', 'mdeg', 'mm', 'mdeg'};
col_scales = [1000, 1000*180/pi, 1000, 1000*180/pi];

fprintf('\n  %-10s %12s  %-8s\n', 'Parameter', 'True \Delta', 'Units');
fprintf('  %s\n', repmat('-', 1, 34));

for i = 1:n_joints
    for c = 1:4
        lbl = sprintf('J%d-%s', i, labels{c});
        val = delta_true(i, c) * col_scales(c);
        fprintf('  %-10s %12.4f  %-8s\n', lbl, val, col_units{c});
    end
end
