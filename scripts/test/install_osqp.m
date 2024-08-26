% 先下载osqp.tar.gz，随后将本函数与osqp.tar.gz放在同一个文件夹中
% 在命令行中输入install_osqp，执行安装
% 若显示OSQP is successfully installed!则安装完成

function install_osqp
% Install the OSQP solver Matlab interface

% Get current operating system
if ispc
    platform = 'windows';
elseif ismac
    platform = 'mac';
elseif isunix
    platform = 'linux';
end

fprintf('Unpacking...');
untar('osqp.tar.gz','osqp')
fprintf('\t\t\t\t\t[done]\n');

fprintf('Updating path...');
cd osqp
addpath(genpath(pwd));
savepath
cd ..
fprintf('\t\t\t\t[done]\n');

fprintf('Deleting temporary files...');
delete('osqp.tar.gz');
fprintf('\t\t\t[done]\n');

fprintf('OSQP is successfully installed!\n');