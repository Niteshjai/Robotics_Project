function draw_frame(ax,T,len)

orig = T(1:3,4);

x_axis = T(1:3,1)*len;
y_axis = T(1:3,2)*len;
z_axis = T(1:3,3)*len;

quiver3(ax,orig(1),orig(2),orig(3), ...
    x_axis(1),x_axis(2),x_axis(3),0,'r','LineWidth',1.5);

quiver3(ax,orig(1),orig(2),orig(3), ...
    y_axis(1),y_axis(2),y_axis(3),0,'g','LineWidth',1.5);

quiver3(ax,orig(1),orig(2),orig(3), ...
    z_axis(1),z_axis(2),z_axis(3),0,'b','LineWidth',1.5);

end