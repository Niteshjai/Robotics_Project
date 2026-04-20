function pose = pose_from_matrix(T)

xyz  = T(1:3, 4);
rpy  = rotation_to_rpy(T(1:3, 1:3));
pose = [xyz; rpy];
end
