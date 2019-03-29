function [A, pos, q] = estimate_pose_test3(sensor, varargin)
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
% load('studentdata1.mat');
% sensor = data(1);
% sensor.id

%%%April Tag Parameters%%%
side_len = 0.152;
gap_1 = 0.152;
gap_2 = 0.178;
gap_offset = gap_2 - gap_1;

%%%Camera Calibration%%%
% Intrinstic
K = [311.0520 0        201.8724;
     0        311.3885 113.6210;
     0        0        1]; 
 
%Rotation 
R_c2i = [sin(pi/4) -cos(pi/4) 0;
        -cos(pi/4) -sin(pi/4) 0;
         0          0        -1];

%Transform
T_c2i = [0.04*sin(pi/4);
        -0.04*cos(pi/4);
        -0.03];

%%%Cordinates in World Frame and Image Frame%%%
n = size(sensor.id,2);
p_w = ones(3,5,n);
p_i = ones(3,5,n);

if sensor.is_ready
    for i = 1 : n
        %%%image cordinates%%%
        p_i(1:2,:,i) = [sensor.p0(:,i) sensor.p1(:,i) sensor.p2(:,i) sensor.p3(:,i) sensor.p4(:,i)];
        %%%World cordinates%%%
        x_i = mod(sensor.id(i),12);
        y_i = floor(sensor.id(i)/12); %position of the April Tag
        corner_x = [1 2 2 0 0]; %x offset
        corner_y = [1 0 2 2 0]; %y offset
        p_w(1,:,i) = (4*x_i*ones(1,5) + corner_x)*(side_len/2);
        p_w_temp = (4*y_i*ones(1,5) + corner_y)*(side_len/2);
        %%%judge in which column
        if sensor.id(i) < 36
            p_w(2,:,i) =  p_w_temp;
        elseif sensor.id(i) < 72
            p_w(2,:,i) =  p_w_temp + ones(1,5)*gap_offset;
        else
            p_w(2,:,i) =  p_w_temp + 2*ones(1,5)*gap_offset;
        end
        
    end
    
    %%% Camera Frame %%%
    p_c = zeros(3,5,n);
    for ii = 1 : n
        p_c(:,:,ii) = inv(K)*p_i(:,:,ii);
    end
    
    %%%Calculate A%%%
    A = zeros(10*n,9);
    for j = 1 : n
        a = zeros(10,9);
        for jj = 1 : 5
            X_w = p_w(1,jj,j);
            Y_w = p_w(2,jj,j);
            X_c = p_c(1,jj,j);
            Y_c = p_c(2,jj,j);
            a(2*jj-1:2*jj,:) = [-X_w -Y_w -1 0 0 0 X_w*X_c Y_w*X_c X_c;
                                0 0 0 -X_w -Y_w -1 X_w*Y_c Y_w*Y_c Y_c];
        end
        A(10*(j-1)+1:10*j,:) = a;
    end
    
    %%%Compute H%%%
    [U S V] = svd(A);
    h = V(:,9)/V(9,9);
    H = [h(1,1) h(2,1) h(3,1);
         h(4,1) h(5,1) h(6,1);
         h(7,1) h(8,1) h(9,1)];
     
    %%%Compute R T%%% 
    h1 = H(:,1)/norm(H(:,1));
    h2 = H(:,2)/norm(H(:,2));
    h3 = cross(h1,h2);
    R = [h1 h2 h3];
    [U1 S1 V1] = svd(R);
    R = U1*[1,0,0;0,1,0;0,0,det(U1*V1')]*V1';
    T = H(:,3)/norm(H(:,1));
    
    %%%From World to Image%%%
    H_w2i = [R_c2i T_c2i;0 0 0 1]*[R T;0 0 0 1];
    H_i2w = inv(H_w2i);
    R_i2w = H_i2w(1:3,1:3);
    T_i2w = H_i2w(1:3,4);
    
    %%%position%%%
    pos = T_i2w;
    
    %%%quaternion%%%
    t = R_i2w(1,1) + R_i2w(2,2) + R_i2w(3,3);
    psi = acos((t-1)/2);
    u_hat = (1/(2*sin(psi)))*(R_i2w - R_i2w');
    u = [u_hat(3,2);u_hat(1,3);u_hat(2,1)];
    q = [cos(psi/2);sin(psi/2)*u];
else
    pos = zeros(3,0);
    q = zeros(4,0);
end
        
end
