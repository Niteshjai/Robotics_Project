function robot = create_robot_tree(dh_params, name)
    if exist('rigidBodyTree', 'builtin') || exist('rigidBodyTree', 'file')
        robot = build_with_rigid_body_tree(dh_params, name);
    elseif exist('SerialLink', 'file')
        robot = build_with_serial_link(dh_params, name);
    else
        error(['No robotics toolbox found. ' ...
               'Install Robotics System Toolbox (rigidBodyTree) ' ...
               'or Peter Corke Toolbox (SerialLink).']);
    end
end

function robot = build_with_rigid_body_tree(dh_params, name)
    n = size(dh_params, 1);

    try
        robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', n);
    catch
        robot = rigidBodyTree;
        robot.DataFormat = 'column';
    end

    parent_name = robot.BaseName;

    for i = 1 : n
        a     = dh_params(i, 1);
        d     = dh_params(i, 2);
        alpha = dh_params(i, 3);
        theta = dh_params(i, 4);

        body  = rigidBody(sprintf('%s_link%d', name, i));
        joint = rigidBodyJoint(sprintf('%s_joint%d', name, i), 'revolute');

        T = dh_transform(a, d, alpha, theta);
        setFixedTransform(joint, T);

        body.Joint = joint;
        addBody(robot, body, parent_name);
        parent_name = body.Name;
    end
end

function robot = build_with_serial_link(dh_params, name)
    n     = size(dh_params, 1);
    links = Link.empty(0, n);

    for i = 1 : n
        links(i) = Link('d',      dh_params(i, 2), ...
                        'a',      dh_params(i, 1), ...
                        'alpha',  dh_params(i, 3), ...
                        'offset', dh_params(i, 4));
    end

    robot      = SerialLink(links);
    robot.name = name;
end

function T = dh_transform(a, d, alpha, theta)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);

    T = [ct,  -st*ca,   st*sa,  a*ct;
         st,   ct*ca,  -ct*sa,  a*st;
           0,      sa,      ca,     d;
           0,       0,       0,     1];
end