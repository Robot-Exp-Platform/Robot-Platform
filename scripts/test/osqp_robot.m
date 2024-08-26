% by mzh
% 脚本使用MATLAB内置的quadprog()求解器与osqp求解器计算二次规划问题
% 只需要更改待求解文件的路径即可，矩阵H、A、f等的大小会自动判断，结果输出到命令行
% 最后会更加两个求解器的解向量之差的最大元素的绝对值判断两者的解是否相等
% 在MATLAB中安装osqp求解器，详见install_osqp.m
clear;
close all;
clc;

% 读取文件
file_path='./test';
file=fopen(file_path,'rt');
S=textscan(file,'%s','Delimiter','\n');
S=S{1};
% 提取H
idx_start=strfind(S{1},'[');
idx_end=strfind(S{1},']');
temp_cell=strsplit(S{1}(idx_start+1:idx_end-1),',');
temp_size_f=sqrt(length(temp_cell));
H=zeros(temp_size_f,temp_size_f);
for i=1:temp_size_f
    for j=1:temp_size_f
        H(i,j)=str2double(temp_cell{(i-1)*temp_size_f+j});
    end
end
% 提取f
idx_start=strfind(S{2},'[');
idx_end=strfind(S{2},']');
temp_cell=strsplit(S{2}(idx_start+1:idx_end-1),',');
temp_size_f=length(temp_cell);
f=zeros(1,temp_size_f);
for i=1:temp_size_f
    f(1,i)=str2double(temp_cell{i});
end
% 提取l
idx_start=strfind(S{4},'[');
idx_end=strfind(S{4},']');
temp_cell=strsplit(S{4}(idx_start+1:idx_end-1),',');
temp_size_l=length(temp_cell);
l=zeros(1,temp_size_l);
for i=1:temp_size_l
    l(1,i)=str2double(temp_cell{i});
end
% 提取u
idx_start=strfind(S{5},'[');
idx_end=strfind(S{5},']');
temp_cell=strsplit(S{5}(idx_start+1:idx_end-1),',');
u=zeros(1,temp_size_l);
for i=1:temp_size_l
    u(1,i)=str2double(temp_cell{i});
end
% 提取A
idx_start=strfind(S{3},'[');
idx_end=strfind(S{3},']');
temp_cell=strsplit(S{3}(idx_start+1:idx_end-1),',');
A=zeros(temp_size_l,temp_size_f);
for i=1:temp_size_l
    for j=1:temp_size_f
        A(i,j)=str2double(temp_cell{(i-1)*temp_size_f+j});
    end
end
%% 使用osqp求解
% Create an OSQP object
prob = osqp;
% Setup workspace and change alpha parameter
prob.setup(H, f, A, l, u, 'alpha', 1);
% Solve problem
res = prob.solve();
fprintf('\nosqp求解结果x=\n');
x_osqp=res.x

%% 使用quadprog求解
% 处理l<Ax<u，使之满足Ax<b的格式
A=[A;-A];
b=[u -l]';
% 求解
[x,fval] = quadprog(H,f,A,b);
fprintf('\nquadprog求解结果x=\n');
x

%% 计算误差
delta_x=x-x_osqp;
tol=1e-3;
fprintf('判断是否相同，两个求解器的解向量差x_quad-x_osqp的最大值元素<容差?\n');
fprintf('abs(max(_quad-x_osqp))=%.2e\n', abs(max(delta_x)));
if abs(max(delta_x)) < tol
    fprintf('两个求解器的结果相同(容差%.2e)\n',tol);
else
    fprintf('两个求解器求解结果不同(容差%.2e)\n',tol);
end