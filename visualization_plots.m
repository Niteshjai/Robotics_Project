function visualization_plots(nominal, true_params, identified, metrics, result, ...
                              configurations, results_dir)
% VISUALIZATION_PLOTS  Generate and save all four result figures.
%
% Figures:
%   1. DH parameter errors    – grouped bar chart
%   2. Optimiser convergence  – cost at final iteration
%   3. Pose error box plots   – before vs. after calibration
%   4. Robot workspace        – 3-D scatter of end-effector positions
%
% Inputs:
%   nominal        - (n x 4) nominal DH params
%   true_params    - (n x 4) true DH params
%   identified     - (n x 4) identified DH params
%   metrics        - struct from evaluate_calibration
%   result         - struct from identify_dh_params
%   configurations - (m x n) joint configurations
%   results_dir    - path to save PNG files

if ~exist(results_dir, 'dir'), mkdir(results_dir); end

% Colour palette
C_BEFORE = [1.00, 0.48, 0.45];   % coral
C_AFTER  = [0.25, 0.73, 0.31];   % green
C_TRUE   = [0.35, 0.65, 1.00];   % blue
n_joints = size(nominal, 1);

% ======================================================================
% Figure 1 – DH parameter errors (grouped bar chart)
% ======================================================================
fig1 = figure('Name','DH Parameter Errors','Color','w', ...
              'Position',[50 50 1400 380]);
col_names  = {'a  [mm]', '\alpha  [mdeg]', 'd  [mm]', '\theta_{off}  [mdeg]'};
col_scales = [1000, 1000*180/pi, 1000, 1000*180/pi];
joints_lbl = arrayfun(@(x) sprintf('J%d',x), 1:n_joints, 'UniformOutput', false);
x = 1:n_joints;

for c = 1:4
    subplot(1, 4, c);
    nom_err  = (nominal(:,c)    - true_params(:,c)) * col_scales(c);
    iden_err = (identified(:,c) - true_params(:,c)) * col_scales(c);

    b = bar(x, [nom_err, iden_err]);
    b(1).FaceColor = C_BEFORE;  b(1).FaceAlpha = 0.85;
    b(2).FaceColor = C_AFTER;   b(2).FaceAlpha = 0.85;
    set(gca, 'XTick', x, 'XTickLabel', joints_lbl, 'Box', 'off');
    title(col_names{c});  ylabel('Error');  grid on;
    if c == 1
        legend({'Nominal','Identified'}, 'Location','northeast', 'FontSize',8);
    end
    yline(0, '--', 'Color', [0.5 0.5 0.5]);
end
sgtitle('DH Parameter Identification Errors  (relative to True)', 'FontSize', 12);
saveas(fig1, fullfile(results_dir, '1_parameter_errors.png'));

% ======================================================================
% Figure 2 – Convergence summary
% ======================================================================
fig2 = figure('Name','Convergence','Color','w','Position',[50 500 600 380]);
cost_final = 0.5 * sum(result.rms_after^2);
bar(1, result.rms_before*1000, 'FaceColor', C_BEFORE, 'FaceAlpha', 0.85); hold on;
bar(2, result.rms_after*1000,  'FaceColor', C_AFTER,  'FaceAlpha', 0.85);
set(gca, 'XTick', [1 2], 'XTickLabel', {'Before Calib','After Calib'}, 'Box','off');
ylabel('RMS Pose Error [mm/mrad]');
title(sprintf('Calibration Convergence  (%.1f%% improvement, %d iters)', ...
      result.improvement_pct, result.n_iter), 'FontSize', 11);
grid on; legend({'Nominal','Identified'}, 'Location','northeast');
saveas(fig2, fullfile(results_dir, '2_convergence.png'));

% ======================================================================
% Figure 3 – Pose error box plots (before vs. after)
% ======================================================================
ax_lbl  = {'\Deltax [mm]','\Deltay [mm]','\Deltaz [mm]', ...
           '\Deltaroll [mdeg]','\Deltapitch [mdeg]','\Deltayaw [mdeg]'};
ax_sc   = [1000, 1000, 1000, 1000*180/pi, 1000*180/pi, 1000*180/pi];

fig3 = figure('Name','Pose Errors','Color','w','Position',[700 50 1100 650]);
for k = 1:6
    subplot(2, 3, k);
    d_before = metrics.errors_nominal(:,k);
    d_after  = metrics.errors_identified(:,k);
    
    d_before = d_before(:);
    d_after  = d_after(:);
    
    x1 = ones(size(d_before));
    x2 = 2 * ones(size(d_after));
    
    scatter(x1, d_before, 15, C_BEFORE, 'filled'); hold on;
    scatter(x2, d_after, 15, C_AFTER, 'filled');
    
    xlim([0.5 2.5]);
    set(gca, 'XTick', [1 2], 'XTickLabel', {'Nominal','Identified'});
    grid on;
    ylabel('Error');
    title(ax_lbl{k});
    rms_b = sqrt(mean(d_before.^2));
    rms_a = sqrt(mean(d_after.^2));
    text(0.97, 0.96, sprintf('RMS: %.3f → %.3f', rms_b, rms_a), ...
         'Units','normalized','HorizontalAlignment','right', ...
         'VerticalAlignment','top','FontSize',8,'Color',[0.45 0.2 0.6]);
end
sgtitle('End-Effector Pose Errors: Before vs. After Calibration','FontSize',12);
saveas(fig3, fullfile(results_dir, '3_pose_errors.png'));

% ======================================================================
% Figure 4 – 3-D workspace scatter
% ======================================================================
pos_nom  = zeros(size(configurations,1), 3);
pos_iden = zeros(size(configurations,1), 3);
for i = 1:size(configurations,1)
    q          = configurations(i,:)';
    T_nom = forward_kinematics(nominal, q);
    pos_nom(i,:) = T_nom(1:3, 4)';

    T_iden = forward_kinematics(identified, q);
    pos_iden(i,:) = T_iden(1:3, 4)';
end

fig4 = figure('Name','Workspace','Color','w','Position',[700 500 800 600]);
scatter3(pos_nom(:,1),  pos_nom(:,2),  pos_nom(:,3),  25, C_BEFORE, 'filled', ...
         'MarkerFaceAlpha', 0.5); hold on;
scatter3(pos_iden(:,1), pos_iden(:,2), pos_iden(:,3), 25, C_AFTER,  'filled', ...
         'MarkerFaceAlpha', 0.5);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('End-Effector Workspace: Nominal vs. Identified FK','FontSize',11);
legend({'Nominal FK','Identified FK'}, 'Location','best','FontSize',9);
grid on; view(35, 30);
saveas(fig4, fullfile(results_dir, '4_workspace.png'));

fprintf('  [Saved] All plots to: %s\n', results_dir);
close all;
end
