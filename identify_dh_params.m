function result = identify_dh_params(nominal_params, configurations, measurements, param_mask, options)

n_joints = size(nominal_params, 1);
n_total  = n_joints * 4;

if nargin < 4 || isempty(param_mask)
    param_mask = true(n_total, 1);
end

if isscalar(param_mask) && param_mask == true
    param_mask = true(n_total, 1);
end

param_mask = logical(param_mask(:));
n_free = sum(param_mask);

if nargin < 5 || isempty(options)
    options = struct();
end
if ~isfield(options, 'MaxIter'),  options.MaxIter = 500;    end
if ~isfield(options, 'FuncTol'),  options.FuncTol = 1e-10;  end
if ~isfield(options, 'StepTol'),  options.StepTol = 1e-10;  end
if ~isfield(options, 'Display'),  options.Display = 'iter';  end

fprintf('[Identifier] Free parameters  : %d / %d\n', n_free, n_total);
fprintf('[Identifier] Configurations   : %d\n', size(configurations, 1));
fprintf('[Identifier] Observations     : %d\n', size(measurements, 1) * 6);

function r = residuals(delta_free)
    delta_full = zeros(n_total, 1);
    delta_full(param_mask) = delta_free;
    current_params = nominal_params + reshape(delta_full, 4, n_joints)';
    r = zeros(size(measurements, 1) * 6, 1);
    for ii = 1:size(configurations, 1)
        q      = configurations(ii, :)';
        T      = forward_kinematics(current_params, q);
        p_pred = pose_from_matrix(T);
        
        err = measurements(ii, :)' - p_pred;
        
        err(4:6) = atan2(sin(err(4:6)), cos(err(4:6)));
        
        idx    = (ii-1)*6 + (1:6);
        r(idx) = err;
    end
end

r0         = residuals(zeros(n_free, 1));
rms_before = sqrt(mean(r0.^2));
fprintf('[Identifier] RMS before calib : %.4f mm/mrad\n', rms_before * 1000);

opt = optimoptions('lsqnonlin', ...
    'Algorithm',             'levenberg-marquardt', ...
    'MaxIterations',          options.MaxIter, ...
    'FunctionTolerance',      options.FuncTol, ...
    'StepTolerance',          options.StepTol, ...
    'Display',                options.Display, ...
    'FiniteDifferenceType',   'forward', ...
    'SpecifyObjectiveGradient', false);

x0 = zeros(n_free, 1);
[x_sol, ~, residual, exit_flag, output] = lsqnonlin(@residuals, x0, [], [], opt);

delta_full = zeros(n_total, 1);
delta_full(param_mask) = x_sol;
delta_params     = reshape(delta_full, 4, n_joints)';
identified_params = nominal_params + delta_params;

rms_after = sqrt(mean(residual.^2));
improvement = (rms_before - rms_after) / rms_before * 100;

fprintf('[Identifier] RMS after  calib : %.4f mm/mrad\n', rms_after * 1000);
fprintf('[Identifier] Improvement      : %.1f%%\n', improvement);
fprintf('[Identifier] Iterations       : %d\n', output.iterations);
fprintf('[Identifier] Success (flag)   : %d\n', exit_flag);

result.identified_params = identified_params;
result.delta_params      = delta_params;
result.rms_before        = rms_before;
result.rms_after         = rms_after;
result.improvement_pct   = improvement;
result.n_iter            = output.iterations;
result.success           = exit_flag > 0;
result.exit_flag         = exit_flag;
end
