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
ang = (44.5/45)*pi/4;
% Rotation of camera to robot
R_c2r = [cos(-ang), sin(-ang), 0;
        sin(-ang), -cos(-ang), 0;
         0,          0,        -1];

% Transform of camera to robot
T_c2r = [0.04*cos(ang);
        -0.04*sin(ang);
        -0.03];

% Give out variable used for calculate coordinates in World
gap_norm = 0.152;
gap_spec = 0.178;

if sensor.is_ready
    if size(sensor.id, 2) == 0  % No id capture
        pos = zeros(3,0);
        q = zeros(4,0);
        return
    else  % All things right
       %% Compute matrix A
        % Calculate points coordinates in World frame
        coord_world_p0 = zeros(2, size(sensor.id, 2));
        pt_col = ceil((sensor.id+1) ./ 12);  % 1xN, column num
        pt_row = (sensor.id+1) - (pt_col-1)*12;  % 1xN, row num
        for i = 1: size(sensor.id, 2)
            if pt_col(i) <4  % Column 1-3
                x = (gap_norm/2) + 2*(pt_row(i) - 1)*gap_norm;
                y = (gap_norm/2) + 2*(pt_col(i) - 1)*gap_norm;
                coord_world_p0(:,i) = [x;y];
            elseif pt_col(i) <7  % Column 4-6
                x = (gap_norm/2) + 2*(pt_row(i) - 1)*gap_norm;
                y = (gap_norm/2) + (2*(pt_col(i) - 1)-1)*gap_norm + gap_spec;
                coord_world_p0(:,i) = [x;y];
            else  % Column 7-9
                x = (gap_norm/2) + 2*(pt_row(i) - 1)*gap_norm;
                y = (gap_norm/2) + (2*(pt_col(i) - 1)-2)*gap_norm + gap_spec*2;
                coord_world_p0(:,i) = [x;y];
            end
        end
        coord_world_p1 = coord_world_p0 + [(gap_norm/2); -(gap_norm/2)];
        coord_world_p2 = coord_world_p0 + [(gap_norm/2); (gap_norm/2)];
        coord_world_p3 = coord_world_p0 + [-(gap_norm/2); (gap_norm/2)];
        coord_world_p4 = coord_world_p0 + [-(gap_norm/2); -(gap_norm/2)];
        
        coord_img_p0 = sensor.p0 ./ 1;
        coord_img_p1 = sensor.p1 ./ 1;
        coord_img_p2 = sensor.p2 ./ 1;
        coord_img_p3 = sensor.p3 ./ 1;
        coord_img_p4 = sensor.p4 ./ 1;
        
        % Calculate matrix A
        A = zeros(10*size(sensor.id, 2), 9);  % Should be 10Nx9
        for n = 1: size(sensor.id, 2)
            A(10*(n-1)+1:10*n, :) = ...
                [coord_world_p0(1,n), coord_world_p0(2,n), 1, 0, 0, 0, -coord_img_p0(1,n)*coord_world_p0(1,n), -coord_img_p0(1,n)*coord_world_p0(2,n), -coord_img_p0(1,n);
                 0, 0, 0, coord_world_p0(1,n), coord_world_p0(2,n), 1, -coord_img_p0(2,n)*coord_world_p0(1,n), -coord_img_p0(2,n)*coord_world_p0(2,n), -coord_img_p0(2,n);
                 
                 coord_world_p1(1,n), coord_world_p1(2,n), 1, 0, 0, 0, -coord_img_p1(1,n)*coord_world_p1(1,n), -coord_img_p1(1,n)*coord_world_p1(2,n), -coord_img_p1(1,n);
                 0, 0, 0, coord_world_p1(1,n), coord_world_p1(2,n), 1, -coord_img_p1(2,n)*coord_world_p1(1,n), -coord_img_p1(2,n)*coord_world_p1(2,n), -coord_img_p1(2,n);
                 
                 coord_world_p2(1,n), coord_world_p2(2,n), 1, 0, 0, 0, -coord_img_p2(1,n)*coord_world_p2(1,n), -coord_img_p2(1,n)*coord_world_p2(2,n), -coord_img_p2(1,n);
                 0, 0, 0, coord_world_p2(1,n), coord_world_p2(2,n), 1, -coord_img_p2(2,n)*coord_world_p2(1,n), -coord_img_p2(2,n)*coord_world_p2(2,n), -coord_img_p2(2,n);
                 
                 coord_world_p3(1,n), coord_world_p3(2,n), 1, 0, 0, 0, -coord_img_p3(1,n)*coord_world_p3(1,n), -coord_img_p3(1,n)*coord_world_p3(2,n), -coord_img_p3(1,n);
                 0, 0, 0, coord_world_p3(1,n), coord_world_p3(2,n), 1, -coord_img_p3(2,n)*coord_world_p3(1,n), -coord_img_p3(2,n)*coord_world_p3(2,n), -coord_img_p3(2,n);
                 
                 coord_world_p4(1,n), coord_world_p4(2,n), 1, 0, 0, 0, -coord_img_p4(1,n)*coord_world_p4(1,n), -coord_img_p4(1,n)*coord_world_p4(2,n), -coord_img_p4(1,n);
                 0, 0, 0, coord_world_p4(1,n), coord_world_p4(2,n), 1, -coord_img_p4(2,n)*coord_world_p4(1,n), -coord_img_p4(2,n)*coord_world_p4(2,n), -coord_img_p4(2,n);];
        end
        
       %% Calculate matrix h and matrix H
        [~,~,V] = svd(A);
        h = V(:, end) / V(end, end);  % 9x1
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
        T = T_temp / ((norm(RRT(:, 1)) + norm(RRT(:, 2))) / 2);  % 3x1
        
       %% Calculate pos and q
        % From world to robot
        H_r2w = inv([R_c2r, T_c2r; 0, 0, 0, 1] * [R, T; 0, 0, 0, 1]);
        R_r2w = H_r2w(1:3, 1:3);
        T_r2w = H_r2w(1:3, 4);
        % Calculate pos
        pos = T_r2w;
        % Calculate quaternion
        phi = acos((R_r2w(1,1) + R_r2w(2,2) + R_r2w(3,3)-1)/2);
        u_hat = (1/(2*sin(phi)))*(R_r2w - R_r2w');
        u = [u_hat(3,2); u_hat(1,3); u_hat(2,1)];
        q = [cos(phi/2); sin(phi/2) * u];
        
    end
else
    pos = zeros(3,0);
    q = zeros(4,0);
end

end

