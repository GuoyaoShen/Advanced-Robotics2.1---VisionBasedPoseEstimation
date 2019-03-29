% This is function is used to calculate points coordinates in world frame
% data saved as 2xN for N points, first row x, second row y.
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
% Tag ids:
% [0, 12, 24, 36, 48, 60, 72, 84,  96;
%  1, 13, 25, 37, 49, 61, 73, 85,  97;
%  2, 14, 26, 38, 50, 62, 74, 86,  98;
%  3, 15, 27, 39, 51, 63, 75, 87,  99;
%  4, 16, 28, 40, 52, 64, 76, 88, 100;
%  5, 17, 29, 41, 53, 65, 77, 89, 101;
%  6, 18, 30, 42, 54, 66, 78, 90, 102;
%  7, 19, 31, 43, 55, 67, 79, 91, 103;
%  8, 20, 32, 44, 56, 68, 80, 92, 104;
%  9, 21, 33, 45, 57, 69, 81, 93, 105;
% 10, 22, 34, 46, 58, 70, 82, 94, 106;
% 11, 23, 35, 47, 59, 71, 83, 95, 107];

p1_coord_world = zeros(2, 108);
p2_coord_world = zeros(2, 108);
p3_coord_world = zeros(2, 108);
p4_coord_world = zeros(2, 108);
p0_coord_world = zeros(2, 108);

%% Fill the value
p1_coord_world(2, 1:12) = 0;  %1
p2_coord_world(2, 1:12) = 0.152;
p3_coord_world(2, 1:12) = 0.152;
p4_coord_world(2, 1:12) = 0;
for i = 1:12
    p1_coord_world(1, i) = 0.152*i + 0.152*(i-1);
    p2_coord_world(1, i) = 0.152*i + 0.152*(i-1);
    p3_coord_world(1, i) = 0.152*2*(i-1);
    p4_coord_world(1, i) = 0.152*2*(i-1);
end

p1_coord_world(2, 13:24) = 0.152*2;  %2
p2_coord_world(2, 13:24) = 0.152*3;
p3_coord_world(2, 13:24) = 0.152*3;
p4_coord_world(2, 13:24) = 0.152*2;
for i = 1:12
    p1_coord_world(1, 12+i) = 0.152*i + 0.152*(i-1);
    p2_coord_world(1, 12+i) = 0.152*i + 0.152*(i-1);
    p3_coord_world(1, 12+i) = 0.152*2*(i-1);
    p4_coord_world(1, 12+i) = 0.152*2*(i-1);
end

p1_coord_world(2, 25:36) = 0.152*4;  %3
p2_coord_world(2, 25:36) = 0.152*5;
p3_coord_world(2, 25:36) = 0.152*5;
p4_coord_world(2, 25:36) = 0.152*4;
for i = 1:12
    p1_coord_world(1, 24+i) = 0.152*i + 0.152*(i-1);
    p2_coord_world(1, 24+i) = 0.152*i + 0.152*(i-1);
    p3_coord_world(1, 24+i) = 0.152*2*(i-1);
    p4_coord_world(1, 24+i) = 0.152*2*(i-1);
end

p1_coord_world(2, 37:48) = 0.152*5+0.178;  %4
p2_coord_world(2, 37:48) = 0.152*6+0.178;
p3_coord_world(2, 37:48) = 0.152*6+0.178;
p4_coord_world(2, 37:48) = 0.152*5+0.178;
for i = 1:12
    p1_coord_world(1, 36+i) = 0.152*i + 0.152*(i-1);
    p2_coord_world(1, 36+i) = 0.152*i + 0.152*(i-1);
    p3_coord_world(1, 36+i) = 0.152*2*(i-1);
    p4_coord_world(1, 36+i) = 0.152*2*(i-1);
end

p1_coord_world(2, 49:60) = 0.152*7+0.178;  %5
p2_coord_world(2, 49:60) = 0.152*8+0.178;
p3_coord_world(2, 49:60) = 0.152*8+0.178;
p4_coord_world(2, 49:60) = 0.152*7+0.178;
for i = 1:12
    p1_coord_world(1, 48+i) = 0.152*i + 0.152*(i-1);
    p2_coord_world(1, 48+i) = 0.152*i + 0.152*(i-1);
    p3_coord_world(1, 48+i) = 0.152*2*(i-1);
    p4_coord_world(1, 48+i) = 0.152*2*(i-1);
end

p1_coord_world(2, 61:72) = 0.152*9+0.178;  %6
p2_coord_world(2, 61:72) = 0.152*10+0.178;
p3_coord_world(2, 61:72) = 0.152*10+0.178;
p4_coord_world(2, 61:72) = 0.152*9+0.178;
for i = 1:12
    p1_coord_world(1, 60+i) = 0.152*i + 0.152*(i-1);
    p2_coord_world(1, 60+i) = 0.152*i + 0.152*(i-1);
    p3_coord_world(1, 60+i) = 0.152*2*(i-1);
    p4_coord_world(1, 60+i) = 0.152*2*(i-1);
end

p1_coord_world(2, 73:84) = 0.152*10+0.178*2;  %7
p2_coord_world(2, 73:84) = 0.152*11+0.178*2;
p3_coord_world(2, 73:84) = 0.152*11+0.178*2;
p4_coord_world(2, 73:84) = 0.152*10+0.178*2;
for i = 1:12
    p1_coord_world(1, 72+i) = 0.152*i + 0.152*(i-1);
    p2_coord_world(1, 72+i) = 0.152*i + 0.152*(i-1);
    p3_coord_world(1, 72+i) = 0.152*2*(i-1);
    p4_coord_world(1, 72+i) = 0.152*2*(i-1);
end

p1_coord_world(2, 85:96) = 0.152*12+0.178*2;  %8
p2_coord_world(2, 85:96) = 0.152*13+0.178*2;
p3_coord_world(2, 85:96) = 0.152*13+0.178*2;
p4_coord_world(2, 85:96) = 0.152*12+0.178*2;
for i = 1:12
    p1_coord_world(1, 84+i) = 0.152*i + 0.152*(i-1);
    p2_coord_world(1, 84+i) = 0.152*i + 0.152*(i-1);
    p3_coord_world(1, 84+i) = 0.152*2*(i-1);
    p4_coord_world(1, 84+i) = 0.152*2*(i-1);
end

p1_coord_world(2, 97:108) = 0.152*14+0.178*2;  %9
p2_coord_world(2, 97:108) = 0.152*15+0.178*2;
p3_coord_world(2, 97:108) = 0.152*15+0.178*2;
p4_coord_world(2, 97:108) = 0.152*14+0.178*2;
for i = 1:12
    p1_coord_world(1, 96+i) = 0.152*i + 0.152*(i-1);
    p2_coord_world(1, 96+i) = 0.152*i + 0.152*(i-1);
    p3_coord_world(1, 96+i) = 0.152*2*(i-1);
    p4_coord_world(1, 96+i) = 0.152*2*(i-1);
end

p0_coord_world = p4_coord_world + 0.076;

coord_world = [p0_coord_world; p1_coord_world; p2_coord_world; p3_coord_world; p4_coord_world];
save('coord_world.mat', 'coord_world');

