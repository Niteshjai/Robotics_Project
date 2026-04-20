function visualize_robot_comparison(nominal, true_dh, identified, configs)
% VISUALIZE_ROBOT_COMPARISON  Plots 3D models of nominal, true, and identified robots.
%
%   nominal:    (n x 4) matrix
%   true_dh:    (n x 4) matrix
%   identified: (n x 4) matrix
%   configs:    (m x n) matrix of joint configurations to cycle through

    fprintf('  Generating 3D Visualization...\n');

    % Input validation
    assert(size(configs, 2) == size(nominal, 1), ...
        'configs columns (%d) must match robot DOF (%d)', ...
        size(configs, 2), size(nominal, 1));

    % Create robot trees
    robot_nom  = create_robot_tree(nominal,    'Nominal');
    robot_true = create_robot_tree(true_dh,    'True');
    robot_iden = create_robot_tree(identified, 'Identified');

    % Setup Figure
    fig = figure('Name', 'Robot DH Comparison', 'Color', 'w', ...
                 'NumberTitle', 'off', 'Position', [100 100 1000 800]);
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 3);
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
    title(ax, 'Robot DH Calibration Comparison');

    q0 = configs(1, :)';

    % Plot all three robots
    childrenBefore = ax.Children;
    show_robot(robot_nom, q0, ax, false);
    newChildren = setdiff(ax.Children, childrenBefore, 'stable');
    patches = findobj(newChildren, 'Type', 'Patch');
    if ~isempty(patches)
        set(patches, 'FaceColor', [0.2 0.2 0.8], 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1);
    end

    childrenBefore = ax.Children;
    show_robot(robot_true, q0, ax, true);
    newChildren = setdiff(ax.Children, childrenBefore, 'stable');
    patches = findobj(newChildren, 'Type', 'Patch');
    if ~isempty(patches)
        set(patches, 'FaceColor', [0.2 0.8 0.2], 'FaceAlpha', 0.3);
    end

    childrenBefore = ax.Children;
    show_robot(robot_iden, q0, ax, false);
    newChildren = setdiff(ax.Children, childrenBefore, 'stable');
    patches = findobj(newChildren, 'Type', 'Patch');
    if ~isempty(patches)
        set(patches, 'FaceColor', [0.8 0.2 0.2], 'FaceAlpha', 0.6);
    end

    % Legend using dummy handles
    h1 = plot3(ax, NaN, NaN, NaN, 's', 'MarkerFaceColor', [0.2 0.2 0.8], 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
    h2 = plot3(ax, NaN, NaN, NaN, 's', 'MarkerFaceColor', [0.2 0.8 0.2], 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
    h3 = plot3(ax, NaN, NaN, NaN, 's', 'MarkerFaceColor', [0.8 0.2 0.2], 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
    legend(ax, [h1, h2, h3], {'Nominal', 'True (Ground Truth)', 'Identified (Calibrated)'}, 'Location', 'best');

    % Slider for multiple configs
    if size(configs, 1) > 1
        uicontrol('Style', 'text', 'Position', [20 20 200 20], ...
                  'String', 'Use Slider to change Configuration');

        smallStep = 1 / (size(configs, 1) - 1);
        largeStep = min(10 / (size(configs, 1) - 1), 1.0);

        s = uicontrol('Style', 'slider', ...
                      'Min', 1, 'Max', size(configs, 1), 'Value', 1, ...
                      'SliderStep', [smallStep, largeStep], ...
                      'Position', [230 20 400 20]);

        s.Callback = @(src, ~) update_viz(src, robot_nom, robot_true, robot_iden, configs, ax);
    end
end

% -----------------------------------------------------------------------

function update_viz(src, robot_nom, robot_true, robot_iden, configs, ax)
    idx = round(get(src, 'Value'));
    q   = configs(idx, :)';

    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 3);
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');

    childrenBefore = ax.Children;
    show_robot(robot_nom, q, ax, false);
    newChildren = setdiff(ax.Children, childrenBefore, 'stable');
    patches = findobj(newChildren, 'Type', 'Patch');
    if ~isempty(patches)
        set(patches, 'FaceColor', [0.2 0.2 0.8], 'FaceAlpha', 0.1);
    end

    childrenBefore = ax.Children;
    show_robot(robot_true, q, ax, true);
    newChildren = setdiff(ax.Children, childrenBefore, 'stable');
    patches = findobj(newChildren, 'Type', 'Patch');
    if ~isempty(patches)
        set(patches, 'FaceColor', [0.2 0.8 0.2], 'FaceAlpha', 0.2);
    end

    childrenBefore = ax.Children;
    show_robot(robot_iden, q, ax, false);
    newChildren = setdiff(ax.Children, childrenBefore, 'stable');
    patches = findobj(newChildren, 'Type', 'Patch');
    if ~isempty(patches)
        set(patches, 'FaceColor', [0.8 0.2 0.2], 'FaceAlpha', 0.5);
    end

    h1 = plot3(ax, NaN, NaN, NaN, 's', 'MarkerFaceColor', [0.2 0.2 0.8], 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
    h2 = plot3(ax, NaN, NaN, NaN, 's', 'MarkerFaceColor', [0.2 0.8 0.2], 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
    h3 = plot3(ax, NaN, NaN, NaN, 's', 'MarkerFaceColor', [0.8 0.2 0.2], 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
    legend(ax, [h1, h2, h3], {'Nominal', 'True (Ground Truth)', 'Identified (Calibrated)'}, 'Location', 'best');

    title(ax, sprintf('Configuration %d / %d', idx, size(configs, 1)));
    drawnow;
end

% -----------------------------------------------------------------------

function show_robot(robot, q, ax, show_frames)
% Handles both rigidBodyTree and SerialLink automatically
    if isa(robot, 'rigidBodyTree')
        if show_frames
            show(robot, q, 'Parent', ax, 'Frames', 'on',  'PreservePlot', true);
        else
            show(robot, q, 'Parent', ax, 'Frames', 'off', 'PreservePlot', true);
        end
    elseif isa(robot, 'SerialLink')
        axes(ax);
        robot.plot(q', 'noshadow', 'noname', 'nowrist', 'nojaxes');
    else
        error('Unknown robot type: %s', class(robot));
    end
end