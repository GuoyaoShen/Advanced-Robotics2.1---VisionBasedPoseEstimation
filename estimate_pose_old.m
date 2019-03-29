function [pos, q] = estimate_pose(sensor, varargin)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include:
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings (you should not use these in this phase)
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags, if no tags are present return empty
%                arrays for pos, q
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
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

% pos = zeros(3,0);
% q = zeros(4,0);

K = [311.0520, 0, 201.8724;
     0, 311.3885, 113.6210;
     0, 0, 1];
% Load world map
coord_world = cell2mat(struct2cell(load('coord_world.mat')));

%=== Check if sensor ready ===
if sensor.is_ready == 0
    warning('SENSOR IS NOT READY !!!');
end

if size(sensor.id) == 0
    pos = [];
    q = [];
    return
end

%% Calculate matrix H
% Calculate matrix A
coord_img = [sensor.p0; sensor.p1; sensor.p2; sensor.p3; sensor.p4];

% matrix1 = [];
% matrix2 = [];

% for i = 1:5
%    % should be 10nx9
%     matrix1 = [matrix1;
%                (coord_world((2*i-1), sensor.id+1))', (coord_world((2*i), sensor.id+1))', ones(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), -(coord_img((2*i-1), :))' .* (coord_world((2*i-1), sensor.id+1))', -(coord_img((2*i-1), :))' .* (coord_world((2*i), sensor.id+1))', -(coord_img((2*i-1), :))'];
%     matrix2 = [matrix2;
%                zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), (coord_world((2*i-1), sensor.id+1))', (coord_world((2*i), sensor.id+1))', ones(size(sensor.id, 2), 1), -(coord_img((2*i), :))' .* (coord_world((2*i-1), sensor.id+1))', -(coord_img((2*i), :))' .* (coord_world((2*i), sensor.id+1))', -(coord_img((2*i), :))'];
% end

matrix11 = [(coord_world((1), sensor.id+1))', (coord_world((2), sensor.id+1))', ones(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), -(coord_img((1), :))' .* (coord_world((1), sensor.id+1))', -(coord_img((1), :))' .* (coord_world((2), sensor.id+1))', -(coord_img((1), :))'];
matrix21 = [zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), (coord_world((1), sensor.id+1))', (coord_world((2), sensor.id+1))', ones(size(sensor.id, 2), 1), -(coord_img((2), :))' .* (coord_world((1), sensor.id+1))', -(coord_img((2), :))' .* (coord_world((2), sensor.id+1))', -(coord_img((2), :))'];

matrix12 = [(coord_world((3), sensor.id+1))', (coord_world((4), sensor.id+1))', ones(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), -(coord_img((3), :))' .* (coord_world((3), sensor.id+1))', -(coord_img((3), :))' .* (coord_world((4), sensor.id+1))', -(coord_img((3), :))'];
matrix22 = [zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), (coord_world((3), sensor.id+1))', (coord_world((4), sensor.id+1))', ones(size(sensor.id, 2), 1), -(coord_img((4), :))' .* (coord_world((3), sensor.id+1))', -(coord_img((4), :))' .* (coord_world((4), sensor.id+1))', -(coord_img((4), :))'];

matrix13 = [(coord_world((5), sensor.id+1))', (coord_world((6), sensor.id+1))', ones(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), -(coord_img((5), :))' .* (coord_world((5), sensor.id+1))', -(coord_img((5), :))' .* (coord_world((6), sensor.id+1))', -(coord_img((5), :))'];
matrix23 = [zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), (coord_world((5), sensor.id+1))', (coord_world((6), sensor.id+1))', ones(size(sensor.id, 2), 1), -(coord_img((6), :))' .* (coord_world((5), sensor.id+1))', -(coord_img((6), :))' .* (coord_world((6), sensor.id+1))', -(coord_img((6), :))'];

matrix14 = [(coord_world((7), sensor.id+1))', (coord_world((8), sensor.id+1))', ones(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), -(coord_img((7), :))' .* (coord_world((7), sensor.id+1))', -(coord_img((7), :))' .* (coord_world((8), sensor.id+1))', -(coord_img((7), :))'];
matrix24 = [zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), (coord_world((7), sensor.id+1))', (coord_world((8), sensor.id+1))', ones(size(sensor.id, 2), 1), -(coord_img((8), :))' .* (coord_world((7), sensor.id+1))', -(coord_img((8), :))' .* (coord_world((8), sensor.id+1))', -(coord_img((8), :))'];

matrix15 = [(coord_world((9), sensor.id+1))', (coord_world((10), sensor.id+1))', ones(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), -(coord_img((9), :))' .* (coord_world((9), sensor.id+1))', -(coord_img((9), :))' .* (coord_world((10), sensor.id+1))', -(coord_img((9), :))'];
matrix25 = [zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), zeros(size(sensor.id, 2), 1), (coord_world((9), sensor.id+1))', (coord_world((10), sensor.id+1))', ones(size(sensor.id, 2), 1), -(coord_img((10), :))' .* (coord_world((9), sensor.id+1))', -(coord_img((10), :))' .* (coord_world((10), sensor.id+1))', -(coord_img((10), :))'];

matrix1 = [matrix11; matrix12; matrix13; matrix14; matrix15];
matrix2 = [matrix21; matrix22; matrix23; matrix24; matrix25];
% Rearrange rows of matrix A
[a,b] = size(matrix1);
A = reshape([matrix1';matrix2'], b, 2*a)';

%% Calculate matrix h and matrix H
[~,~,V] = svd(A);
h = V(:, end) ./ V(end, end);  % 9x1
H = [h(1:3)'; h(4:6)'; h(7:9)'];  % 3x3

%% Calculate matrix (r1, r2, T)
RRT = K \ H;
r1 = RRT(:, 1);
r2 = RRT(:, 2);
T_temp = RRT(:, 3);

%% Calculate R and T
R_temp = [r1, r2, cross(r1, r2)];
[Ur,~,Vr] = svd(R_temp);
R = Ur * [1, 0, 0; 0, 1, 0; 0, 0, det(Ur*(Vr'))] * Vr';  % 3x3
T = T_temp ./ norm(r1);  % 3x1

%% Calculate pos and q
Homo_trans = [R,T;0,0,0,1];
PosInCam = [-0.04; 0.0; -0.03; 1];
pos = (Homo_trans) \ PosInCam;
pos(4)=[];
RotCamtoQuad = rotx(180)*rotz(45);
RotQuadinWorld = R*RotCamtoQuad;
q = (rot2quater(RotQuadinWorld))';

end


%% Other functions
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
