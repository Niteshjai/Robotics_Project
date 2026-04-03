%% main.m
% Entry point for the Robot DH Parameter Identification project (MATLAB).
%
% Pipeline:
%   1. Create calibration dataset
%   2. Split into training (80%) and test (20%) sets
%   3. Identify DH parameters from training set using Levenberg-Marquardt
%   4. Evaluate on test set
%   5. Print summary tables
%   6. Save result plots to results/
%
% References:
%   Mooring et al. (1991). Fundamentals of Manipulator Calibration.
%   Roth et al. (1987). IEEE J. Robotics Automation 3(5), 377-385.

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

% -------------------------------------------------------------------------
% Step 4 – DH Parameter Identification (Levenberg-Marquardt)
% -------------------------------------------------------------------------
fprintf('\n────────────────────────────────────────────────────────────────────────\n');
fprintf('  Step 3 – DH Parameter Identification (Levenberg-Marquardt)\n');
fprintf('────────────────────────────────────────────────────────────────────────\n');

% Options for identification
ident_opts.MaxIter = 500;
ident_opts.Display = 'final';

result = identify_dh_params(nominal, train_configs, train_meas, [], ident_opts);
identified = result.identified_params;

% -------------------------------------------------------------------------
% Step 5 – Parameter Comparison Table
% -------------------------------------------------------------------------
fprintf('\n────────────────────────────────────────────────────────────────────────\n');
fprintf('  Step 4 – Parameter Comparison Table\n');
fprintf('────────────────────────────────────────────────────────────────────────\n');

for c = 1:4
    fprintf('\n  ── %s [%s] ──\n', labels{c}, col_units{c});
    fprintf('  %-6s %10s %10s %12s %10s %10s\n', ...
        'Joint', 'Nominal', 'True', 'Identified', '|\DeltaNom|', '|\DeltaIden|');
    fprintf('  %s\n', repmat('-', 1, 62));
    
    for i = 1:n_joints
        nom_v  = nominal(i, c)    * col_scales(c);
        true_v = true_params(i, c) * col_scales(c);
        iden_v = identified(i, c) * col_scales(c);
        err_nom = abs(nom_v  - true_v);
        err_ide = abs(iden_v - true_v);
        
        fprintf('  J%-5d %10.4f %10.4f %12.4f %10.4f %10.4f\n', ...
            i, nom_v, true_v, iden_v, err_nom, err_ide);
    end
end

% -------------------------------------------------------------------------
% Step 6 – Test-Set Pose Error Evaluation
% -------------------------------------------------------------------------
fprintf('\n────────────────────────────────────────────────────────────────────────\n');
fprintf('  Step 5 – Test-Set Pose Error Evaluation\n');
fprintf('────────────────────────────────────────────────────────────────────────\n');

metrics = evaluate_calibration(nominal, identified, true_params, ...
    test_configs, test_meas);

axes_lbl = {'x [mm]', 'y [mm]', 'z [mm]', ...
            'roll [mdeg]', 'pitch [mdeg]', 'yaw [mdeg]'};
axes_sc  = [1000, 1000, 1000, 1000*180/pi, 1000*180/pi, 1000*180/pi];

fprintf('\n  %-14s %12s %15s %10s\n', 'Axis', 'RMS Nominal', 'RMS Identified', 'Reduction');
fprintf('  %s\n', repmat('-', 1, 55));

for k = 1:6
    rn  = metrics.rms_nominal(k);
    ri  = metrics.rms_identified(k);
    pct = (rn - ri) / rn * 100;
    fprintf('  %-14s %12.4f %15.4f %9.1f%%\n', ...
        axes_lbl{k}, rn*axes_sc(k), ri*axes_sc(k), pct);
end

rms_nom_total  = sqrt(mean(metrics.errors_nominal(:).^2));
rms_iden_total = sqrt(mean(metrics.errors_identified(:).^2));
overall_improvement = (rms_nom_total - rms_iden_total) / rms_nom_total * 100;

fprintf('\n  Overall RMS (nominal)    : %.4f mm-equiv\n', rms_nom_total * 1000);
fprintf('  Overall RMS (identified) : %.4f mm-equiv\n', rms_iden_total * 1000);
fprintf('  Overall improvement      : %.1f%%\n', overall_improvement);
fprintf('  Param RMS vs. ground truth: %.2f \mum/\murad\n', metrics.rms_true_err * 1e6);

% -------------------------------------------------------------------------
% Step 7 – Save Plots
% -------------------------------------------------------------------------
fprintf('\n────────────────────────────────────────────────────────────────────────\n');
fprintf('  Step 6 – Saving Result Plots\n');
fprintf('────────────────────────────────────────────────────────────────────────\n');

visualization_plots(nominal, true_params, identified, metrics, result, ...
    configs, RESULTS_DIR);

fprintf('\n────────────────────────────────────────────────────────────────────────\n');
fprintf('  Done\n');
fprintf('────────────────────────────────────────────────────────────────────────\n');

% -------------------------------------------------------------------------
% Step 8 – Automated assertions
% -------------------------------------------------------------------------
assert(overall_improvement > 50.0, 'Expected >50%% improvement');
fprintf('  ✓ Assertion passed: overall improvement > 50%%\n');

assert(rms_iden_total < rms_nom_total, 'Identified RMS must be less than nominal RMS');
fprintf('  ✓ Assertion passed: identified RMS < nominal RMS\n');

fprintf('\n  ✓ All checks passed. Calibration successful!\n\n');
