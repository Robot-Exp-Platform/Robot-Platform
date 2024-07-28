# 使用官方的 Ubuntu 20.04 作为基础镜像
FROM ros:noetic
# FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# 更新包列表并安装必要的包
RUN apt-get update && \
    apt-get install -y \
    curl \
    build-essential \
    git \
    ca-certificates \
    wget \
    x11vnc \
    xfce4 \
    xfce4-goodies \
    tightvncserver \
    dbus-x11 \
    xvfb \
    xauth \
    dbus && \
    rm -rf /var/lib/apt/lists/*

# 设置 VNC 服务器密码
RUN mkdir -p ~/.vnc && echo "yixing312" | vncpasswd -f > ~/.vnc/passwd && chmod 600 ~/.vnc/passwd

# 安装 Miniconda/rust
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    bash Miniconda3-latest-Linux-x86_64.sh -b -p /opt/conda && \
    rm Miniconda3-latest-Linux-x86_64.sh && \
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain nightly -y && \
    touch /root/.Xauthority


# 添加到 PATH
ENV PATH="/opt/conda/bin:${PATH}" 
ENV CONDA_DEFAULT_ENV=myenv 
ENV PATH="/opt/conda/envs/myenv/bin:${PATH}" 
ENV PATH="/root/.cargo/bin:${PATH}"

# 复制文件到镜像中
COPY scripts/requirements.txt /root/
COPY scripts/start-vnc.sh /usr/local/bin/start-vnc.sh

RUN conda create -y --name myenv python=3.12 && \
    /opt/conda/bin/conda run -n myenv pip install -r /root/requirements.txt && \
    echo "source activate myenv" >> ~/.bashrc && \
    dbus-uuidgen > /etc/machine-id

# 设置工作目录
WORKDIR /root

# # 克隆指定的仓库
RUN git clone https://github.com/Robot-Exp-Platform/Robot-Platform.git

# 设置容器启动时默认执行的命令
# CMD ["/usr/local/bin/start-vnc.sh"]
CMD ["sh", "-c", "service dbus start && Xvfb :1 -screen 0 1024x768x16 & sleep 2 && export DISPLAY=:1 && startxfce4 & sleep 2 && x11vnc -display :1 -forever -shared -rfbport 5900 -auth /root/.Xauthority"]
# CMD ["bash"]

# FROM ubuntu:latest

# # 安装必要的工具和 VNC 服务器
# RUN apt-get update && apt-get install -y \
#     x11vnc \
#     xfce4 \
#     xfce4-goodies \
#     tightvncserver \
#     dbus-x11 \
#     xvfb \
#     xauth \
#     dbus

# # 设置 VNC 服务器密码
# RUN mkdir -p ~/.vnc && echo "your_vnc_password" | vncpasswd -f > ~/.vnc/passwd && chmod 600 ~/.vnc/passwd

# # 创建 .Xauthority 文件
# RUN touch /root/.Xauthority

# # 启动 dbus 服务
# RUN dbus-uuidgen > /etc/machine-id

# # 启动 X 虚拟帧缓冲器 (Xvfb)，XFCE 桌面环境和 VNC 服务器
# CMD ["sh", "-c", "service dbus start && Xvfb :1 -screen 0 1024x768x16 & sleep 2 && export DISPLAY=:1 && startxfce4 & sleep 2 && x11vnc -display :1 -forever -shared -rfbport 5900 -auth /root/.Xauthority"]
