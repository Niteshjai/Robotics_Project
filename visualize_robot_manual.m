function visualize_robot_manual(dh_params, configurations, fig_title)
% VISUALIZE_ROBOT_MANUAL  Plots a robot armature using basic MATLAB graphics.
% No toolboxes required.

    if nargin < 3, fig_title = 'Robot Armature'; end
    
    n_joints = size(dh_params, 1);
    
    fig = figure('Name', fig_title, 'Color', 'w');
    ax = axes('Parent', fig);
    grid on; axis equal; view(3); hold on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(fig_title);

    % Color for the robot
    robot_color = [0.1 0.5 0.8];
    
    % Plot the first configuration
    q = configurations(1, :)';
    plot_armature(ax, dh_params, q, robot_color);

    % Simple slider for interactive configs
    if size(configurations, 1) > 1
        uicontrol('Style', 'slider', 'Min', 1, 'Max', size(configurations, 1), 'Value', 1, ...
                  'Position', [100 20 400 20], ...
                  'Callback', @(src, event) update_plot(src, ax, dh_params, configurations));
    end
end

function update_plot(src, ax, dh_params, configs)
    idx = round(get(src, 'Value'));
    q = configs(idx, :)';
    cla(ax);
    plot_armature(ax, dh_params, q, [0.1 0.5 0.8]);
    title(ax, sprintf('Joint Configuration: %d', idx));
end

function plot_armature(ax, dh_params, q, col)
    n = size(dh_params, 1);
    T_accum = eye(4);
    
    % Track joint positions
    positions = zeros(n+1, 3);
    positions(1, :) = [0 0 0];
    
    for i = 1:n
        a     = dh_params(i, 1);
        alpha = dh_params(i, 2);
        d     = dh_params(i, 3);
        theta_offset = dh_params(i, 4);
        
        % Current joint angle
        th = q(i) + theta_offset;
        
        % Step-by-step DH to show intermediate points if desired, 
        % but for plot we just need final T.
        T_link = [cos(th), -sin(th)*cos(alpha),  sin(th)*sin(alpha), a*cos(th);
                  sin(th),  cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th);
                  0,        sin(alpha),          cos(alpha),         d;
                  0,        0,                   0,                  1];
        
        T_accum = T_accum * T_link;
        positions(i+1, :) = T_accum(1:3, 4)';
        
        % Draw Coordinate Frame at current joint
        draw_frame(ax, T_accum, 0.1);
    end
    
    % Plot links as thick lines
    plot3(ax, positions(:,1), positions(:,2), positions(:,3), '-o', ...
          'Color', col, 'LineWidth', 3, 'MarkerFaceColor', col, 'MarkerSize', 8);
end

function draw_frame(ax, T, len)
    orig = T(1:3, 4);
    x_axis = T(1:3, 1) * len;
    y_axis = T(1:3, 2) * len;
    z_axis = T(1:3, 3) * len;
    
    quiver3(ax, orig(1), orig(2), orig(3), x_axis(1), x_axis(2), x_axis(3), 0, 'r', 'LineWidth', 1.5);
    quiver3(ax, orig(1), orig(2), orig(3), y_axis(1), y_axis(2), y_axis(3), 0, 'g', 'LineWidth', 1.5);
    quiver3(ax, orig(1), orig(2), orig(3), z_axis(1), z_axis(2), z_axis(3), 0, 'b', 'LineWidth', 1.5);
end
