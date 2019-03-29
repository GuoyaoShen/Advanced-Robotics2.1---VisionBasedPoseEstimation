function [pos, q] = estimate_pose(sensor, varargin)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include:
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings (you should not use these in this phase)
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags, if no tags are present return empty
%                arrays for pos, q
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                wwwwwwwwwwwfour corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   q   - 4x1 quaternion of the quadrotor [w, x, y, z] where q = w + x*i + y*j + z*k
R = eye(3); T = [5;6;2];
Homo_trans = [R,T;0,0,0,1];
PosInCam = [-0.04; 0.0; -0.03; 1];
pos = (Homo_trans) \ PosInCam;
pos(4)=[];
RotCamtoQuad = rotx(180)*rotz(45);
RotQuadinWorld = R*RotCamtoQuad;
q = rot2quater(RotQuadinWorld);
disp(pos);
disp(q);

% pos = zeros(3,0);
% q = zeros(4,0);

end

function q = rot2quater(R)
q = zeros(4,0);
tau = trace(R);
cosphi = 0.5*(tau-1);
phi = acos(cosphi);
u_hat = (R-R')/(2*sin(phi));
u = solve_hat(u_hat);
q = [cos(phi/2), sin(phi/2)*u'];
end

function t=solve_hat(t_hat)
    t=[t_hat(3,2); t_hat(1,3); t_hat(2,1)];
end