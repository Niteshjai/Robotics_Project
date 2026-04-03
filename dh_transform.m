function T = dh_transform(a, alpha, d, theta)
% DH_TRANSFORM  Compute 4x4 homogeneous transform for a single DH link.
%
% Modified DH (Craig) convention:
%   T = Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)
%
% Inputs:
%   a     - link length   [m]
%   alpha - link twist    [rad]
%   d     - link offset   [m]
%   theta - joint angle (offset + actual) [rad]
%
% Output:
%   T - 4x4 homogeneous transform from frame i-1 to frame i

ct = cos(theta);  st = sin(theta);
ca = cos(alpha);  sa = sin(alpha);

T = [ct,    -st,     0,    a;
     st*ca,  ct*ca, -sa,  -sa*d;
     st*sa,  ct*sa,  ca,   ca*d;
     0,      0,      0,    1   ];
end
