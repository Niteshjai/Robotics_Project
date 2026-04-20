function plot_armature(ax, dh_params, q, col)

n = size(dh_params,1);
T_accum = eye(4);

positions = zeros(n+1,3);
positions(1,:) = [0 0 0];

for i = 1:n

    a     = dh_params(i,1);
    alpha = dh_params(i,2);
    d     = dh_params(i,3);
    theta_offset = dh_params(i,4);

    th = q(i) + theta_offset;

    T_link = [cos(th), -sin(th)*cos(alpha),  sin(th)*sin(alpha), a*cos(th);
              sin(th),  cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th);
              0,        sin(alpha),          cos(alpha),         d;
              0,        0,                   0,                  1];

    T_accum = T_accum * T_link;

    positions(i+1,:) = T_accum(1:3,4)';

    draw_frame(ax,T_accum,0.08);

end

plot3(ax,positions(:,1),positions(:,2),positions(:,3),'-o', ...
    'LineWidth',4, ...
    'Color',col, ...
    'MarkerFaceColor',col, ...
    'MarkerSize',10);

end