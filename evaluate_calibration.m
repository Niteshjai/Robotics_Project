function metrics = evaluate_calibration(nominal_params, identified_params, ...
                                         true_params, test_configs, test_measurements)

m = size(test_configs, 1);

err_nom  = zeros(m, 6);
err_iden = zeros(m, 6);

for i = 1:m
    q = test_configs(i, :)';

    T_nom   = forward_kinematics(nominal_params,    q);
    T_iden  = forward_kinematics(identified_params, q);

    err_nom(i,  :) = test_measurements(i, :) - pose_from_matrix(T_nom)';
    err_iden(i, :) = test_measurements(i, :) - pose_from_matrix(T_iden)';
    
    err_nom(i,  4:6) = atan2(sin(err_nom(i,  4:6)), cos(err_nom(i,  4:6)));
    err_iden(i, 4:6) = atan2(sin(err_iden(i, 4:6)), cos(err_iden(i, 4:6)));
end

metrics.errors_nominal    = err_nom;
metrics.errors_identified = err_iden;
metrics.rms_nominal       = sqrt(mean(err_nom.^2,  1));
metrics.rms_identified    = sqrt(mean(err_iden.^2, 1));
metrics.rms_true_err      = sqrt(mean((identified_params(:) - true_params(:)).^2));
end
