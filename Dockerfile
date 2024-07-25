# 使用官方的 Ubuntu 20.04 作为基础镜像
FROM ros:noetic
# FROM ubuntu:20.04

# 设置环境变量以避免安装包时的提示
ENV DEBIAN_FRONTEND=noninteractive

# 更新包列表并安装必要的包
RUN apt-get update && \
    apt-get install -y \
    curl \
    build-essential \
    git \
    ca-certificates \
    wget && \
    rm -rf /var/lib/apt/lists/*

# 安装 Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    bash Miniconda3-latest-Linux-x86_64.sh -b -p /opt/conda && \
    rm Miniconda3-latest-Linux-x86_64.sh

# 将 Conda 添加到 PATH
ENV PATH="/opt/conda/bin:${PATH}"

# 创建一个包含 Python 3.12 的 Conda 环境
RUN conda create -y --name myenv python=3.12

# 默认激活 Conda 环境
RUN echo "source activate myenv" >> ~/.bashrc
ENV CONDA_DEFAULT_ENV=myenv
ENV PATH="/opt/conda/envs/myenv/bin:${PATH}"

# 使用 rustup 安装 Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

# 将 Rust 添加到 PATH 环境变量
ENV PATH="/root/.cargo/bin:${PATH}"

# 安装并切换到 Rust 的 nightly 版本
RUN rustup install nightly && rustup default nightly

# 验证安装
# RUN rustc --version && cargo --version && python --version && pip --version && rosversion -d

# 设置工作目录
WORKDIR /usr/src/app

# # 克隆指定的仓库
# RUN git clone https://github.com/Robot-Exp-Platform/Robot-Platform.git

# # 将工作目录设置为克隆的仓库
# WORKDIR /usr/src/app/Robot-Platform

# 默认情况下不执行任何操作
CMD ["bash"]