% add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
% We will only call estimate_pose_handle in the test function.
% Note that unlike project 1 phase 3, thise will only create a function
% handle, but not run the function at all.

estimate_pose_handle1 = @(sensor) estimate_pose(sensor);
estimate_pose_handle2 = @(sensor) estimate_pose_test3(sensor);
result2 = [];
result1 = [];
A_m1 = [];
A_m2 = [];
% 1 guoyao 2 yue yang
for i = 1:size(data,2)
   sensor = data(i);
   [A1, pos, q] = estimate_pose_handle1(sensor);
%    q=q';
   quad_info = [pos;q];
   result1 = [result1, quad_info];
    A_m1 = [A_m1; A1];
end
for i = 1:size(data,2)
   sensor = data(i);
   [A2, pos, q] = estimate_pose_handle2(sensor);
%    q=q';
   quad_info = [pos;q];
   result2 = [result2, quad_info];
   A_m2 = [A_m2; A2];
end
max(max(abs(result1-result2)))