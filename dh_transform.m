function T = dh_transform(a, alpha, d, theta)

% Output:
%   T - 4x4 homogeneous transform from frame i-1 to frame i

ct = cos(theta);  st = sin(theta);
ca = cos(alpha);  sa = sin(alpha);

T = [ct,    -st,     0,    a;
     st*ca,  ct*ca, -sa,  -sa*d;
     st*sa,  ct*sa,  ca,   ca*d;
     0,      0,      0,    1   ];
end
