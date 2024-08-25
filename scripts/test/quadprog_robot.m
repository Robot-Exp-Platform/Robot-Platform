% by mzh
% 只需要更改待求解文件的路径即可，矩阵H、A、f等的大小会自动判断，结果输出到ans.txt
clear;
close all;

% 读取文件
file_path='./test_1';
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
% 处理l<Ax<u，使之满足Ax<b的格式
A=[A;-A];
b=[u -l]';
% 求解
[x,fval] = quadprog(H,f,A,b);
% 写入文件
ans_file=fopen('ans.txt','a');
fprintf(ans_file,'x=[');
for i=1:length(x)-1
    fprintf(ans_file,'%f,',x(i));
end
fprintf(ans_file,'%f]\n',x(length(x)));
fprintf(ans_file,'fval=%f\n',fval);

fclose(file);
fclose(ans_file);