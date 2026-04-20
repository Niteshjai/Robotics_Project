function robot_animation(dh_params, configs)

fig = figure('Color','w');
ax = axes('Parent',fig);

for i = 1:size(configs,1)

    cla(ax);

    q = configs(i,:)';

    plot_armature(ax, dh_params, q, [0 0.45 0.85]);

    xlim(ax,[-0.8 0.8]);
    ylim(ax,[-0.8 0.8]);
    zlim(ax,[0 1.8]);

    axis(ax,'manual');
    axis(ax,'equal');

    view(ax,45,25);

    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    title(ax,sprintf('Frame %d',i));

    drawnow limitrate
    pause(0.01);

end
end