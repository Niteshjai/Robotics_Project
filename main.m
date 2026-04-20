%% main.m
% =========================================================================
% Entry point for the Robot DH Parameter Identification project.
%
% Pipeline
% --------
%   1. Create calibration dataset (nominal DH, true perturbed DH,
%      joint configs, measurements)
%   2. Split into training (80%) and test (20%) sets
%   3. Identify DH parameters from training set using Levenberg-Marquardt
%   4. Evaluate on test set – compare pre/post calibration pose errors
%   5. Print summary table
%   6. Save four result plots to results/


clear; clc;

% -------------------------------------------------------------------------
% Configuration
% -------------------------------------------------------------------------
RESULTS_DIR           = fullfile(fileparts(mfilename('fullpath')), 'results');
N_CONFIGS             = 100;           % total calibration configurations
TEST_FRACTION         = 0.20;          % fraction held out for testing
SEED                  = 42;

% DH perturbation sizes (realistic manufacturing tolerances)
PERTURBATION_STD = struct( ...
    'a',     0.0005, ...               % 0.5 mm
    'alpha', deg2rad(0.02), ...        % 0.02 deg
    'd',     0.0005, ...               % 0.5 mm
    'theta', deg2rad(0.02)  ...        % 0.02 deg
);
MEASUREMENT_NOISE_STD = 1e-4;          % 0.1 mm / ~0.006 deg sensor noise


% =========================================================================
% Step 1 – Generate Calibration Dataset
% =========================================================================
print_header('Step 1 – Generating Calibration Dataset');

[nominal, true_dh, configs, measurements] = data_generator( ...
    N_CONFIGS, PERTURBATION_STD, MEASUREMENT_NOISE_STD, SEED);

n_joints = size(nominal, 1);
fprintf('  Robot            : %d-DOF (Puma-560 inspired)\n', n_joints);
fprintf('  Configurations   : %d  (training + test)\n', N_CONFIGS);
fprintf('  Measurement noise: %.2f mm / %.4f mdeg\n', ...
    MEASUREMENT_NOISE_STD * 1000, ...
    MEASUREMENT_NOISE_STD * 1000 * 180 / pi);
fprintf('  DH perturbation  : a/d ±%.2f mm, α/θ ±%.2f mdeg  (1σ)\n', ...
    PERTURBATION_STD.a  * 1000, ...
    rad2deg(PERTURBATION_STD.alpha) * 1000);


% =========================================================================
% Train / Test Split
% =========================================================================
n_test  = max(1, round(N_CONFIGS * TEST_FRACTION));
n_train = N_CONFIGS - n_test;

rng(SEED + 99);
idx        = randperm(N_CONFIGS);
train_idx  = idx(1 : n_train);
test_idx   = idx(n_train+1 : end);

train_configs = configs(train_idx, :);
train_meas    = measurements(train_idx, :);
test_configs  = configs(test_idx,  :);
test_meas     = measurements(test_idx,  :);

fprintf('\n  Training configs : %d\n', n_train);
fprintf('  Test configs     : %d\n',  n_test);


% =========================================================================
% Step 2 – Print True (Injected) DH Parameter Perturbations
% =========================================================================
print_header('Step 2 – True (Injected) DH Parameter Perturbations');

delta_true      = (true_dh - nominal);            % n_joints x 4
delta_true_flat = delta_true';                     % 4 x n_joints, then flatten col-major
delta_true_flat = delta_true_flat(:);              % (4*n_joints) x 1

param_labels  = {};
col_units_flat  = {};
col_scales_flat = [];
col_names_cycle = {'a', 'α', 'd', 'θ'};
unit_cycle      = {'mm', 'mdeg', 'mm', 'mdeg'};
scale_cycle     = [1000, 1000*180/pi, 1000, 1000*180/pi];

for i = 1:n_joints
    for c = 1:4
        param_labels{end+1}    = sprintf('J%d-%s', i, col_names_cycle{c}); %#ok<SAGROW>
        col_units_flat{end+1}  = unit_cycle{c};                             %#ok<SAGROW>
        col_scales_flat(end+1) = scale_cycle(c);                            %#ok<SAGROW>
    end
end

fprintf('\n  %-10s %12s  %-8s\n', 'Parameter', 'True Δ', 'Units');
fprintf('  %s\n', repmat('-', 1, 34));
for k = 1:length(delta_true_flat)
    fprintf('  %-10s %12.4f  %s\n', ...
        param_labels{k}, delta_true_flat(k) * col_scales_flat(k), col_units_flat{k});
end


% =========================================================================
% Step 3 – DH Parameter Identification (Levenberg-Marquardt)
% =========================================================================
print_header('Step 3 – DH Parameter Identification (Levenberg-Marquardt)');

% We let all 24 parameters be free. 
% Note: Standard DH is redundant for parallel axes (J2-J3), 
% but LM will find a numerical solution.
param_mask = true(n_joints * 4, 1);

result     = identify_dh_params(nominal, train_configs, train_meas, param_mask);
identified = result.identified_params;


% =========================================================================
% Step 4 – Parameter Comparison Table
% =========================================================================
print_header('Step 4 – Parameter Comparison Table');
print_param_table(nominal, true_dh, identified);


% =========================================================================
% Step 5 – Test-Set Pose Error Evaluation
% =========================================================================
print_header('Step 5 – Test-Set Pose Error Evaluation');

metrics = evaluate_calibration(nominal, identified, true_dh, ...
                               test_configs, test_meas);
print_rms_table(metrics);

rms_nom_total  = sqrt(mean(metrics.errors_nominal(:).^2));
rms_iden_total = sqrt(mean(metrics.errors_identified(:).^2));
overall_improvement = (rms_nom_total - rms_iden_total) / rms_nom_total * 100;

fprintf('\n  Overall RMS (nominal)    : %.4f mm-equiv\n', rms_nom_total  * 1000);
fprintf('  Overall RMS (identified) : %.4f mm-equiv\n', rms_iden_total * 1000);
fprintf('  Overall improvement      : %.1f%%\n',        overall_improvement);
fprintf('  Param RMS vs. ground truth: %.4f mm/mrad\n', result.rms_after * 1000);


% =========================================================================
% Step 6– Save Result Plots
% =========================================================================
print_header('Step 6 – Saving Result Plots');

if ~exist(RESULTS_DIR, 'dir')
    mkdir(RESULTS_DIR);
end

visualization_plots(nominal, true_dh, identified, metrics, result, ...
                    configs, RESULTS_DIR);


% =========================================================================
% Step 7 – 3D Visualization
% =========================================================================
print_header('Step 7 – Robot Animation');

robot_animation(identified, test_configs);


% =========================================================================
% Step 8 – Automated Assertions
% =========================================================================
print_header('Done');

fprintf('  All plots saved to: %s\n', RESULTS_DIR);

assert(overall_improvement > 50.0, ...
    sprintf('Expected >50%% improvement, got %.1f%%', overall_improvement));
fprintf('  [OK] Assertion passed: overall improvement > 50%%\n');

assert(rms_iden_total < rms_nom_total, ...
    'Identified RMS must be less than nominal RMS');
fprintf('  [OK] Assertion passed: identified RMS < nominal RMS\n');

fprintf('\n  [OK] All checks passed. Calibration successful!\n\n');


% =========================================================================
%  LOCAL HELPER FUNCTIONS
% =========================================================================

% -------------------------------------------------------------------------
function print_header(title)
% PRINT_HEADER  Print a section header to the console.
    w = 72;
    fprintf('\n%s\n  %s\n%s\n', repmat('─', 1, w), title, repmat('─', 1, w));
end


% -------------------------------------------------------------------------
function print_param_table(nominal, true_dh, identified)
% PRINT_PARAM_TABLE  Print formatted table of DH parameters.
%
%   Inputs: nominal, true_dh, identified — each (n_joints x 4)

    n          = size(nominal, 1);
    col_names  = {'a [mm]', 'α [mdeg]', 'd [mm]', 'θ_off [mdeg]'};
    col_scales = [1000, 1000*180/pi, 1000, 1000*180/pi];

    for c_idx = 1:4
        sc = col_scales(c_idx);
        fprintf('\n  ── %s ──\n', col_names{c_idx});
        fprintf('  %-6s %10s %10s %12s %10s %10s\n', ...
            'Joint', 'Nominal', 'True', 'Identified', '|ΔNom|', '|ΔIden|');
        fprintf('  %s\n', repmat('-', 1, 62));
        for i = 1:n
            nom_v  = nominal(i,    c_idx) * sc;
            true_v = true_dh(i,    c_idx) * sc;
            iden_v = identified(i, c_idx) * sc;
            err_nom = abs(nom_v  - true_v);
            err_ide = abs(iden_v - true_v);
            fprintf('  J%-5d %10.4f %10.4f %12.4f %10.4f %10.4f\n', ...
                i, nom_v, true_v, iden_v, err_nom, err_ide);
        end
    end
end


% -------------------------------------------------------------------------
function print_rms_table(metrics)
% PRINT_RMS_TABLE  Print per-axis RMS before/after calibration.

    axes   = {'x [mm]', 'y [mm]', 'z [mm]', ...
              'roll [mdeg]', 'pitch [mdeg]', 'yaw [mdeg]'};
    scales = [1000, 1000, 1000, ...
              1000*180/pi, 1000*180/pi, 1000*180/pi];

    fprintf('\n  %-14s %12s %15s %10s\n', ...
        'Axis', 'RMS Nominal', 'RMS Identified', 'Reduction');
    fprintf('  %s\n', repmat('-', 1, 55));

    for k = 1:6
        rn  = metrics.rms_nominal(k);
        ri  = metrics.rms_identified(k);
        sc  = scales(k);
        pct = 0;
        if rn > 0
            pct = (rn - ri) / rn * 100;
        end
        fprintf('  %-14s %12.4f %15.4f %9.1f%%\n', ...
            axes{k}, rn*sc, ri*sc, pct);
    end
end