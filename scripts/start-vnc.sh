#!/bin/bash
# 启动虚拟帧缓冲
Xvfb :1 -screen 0 1280x800x16 &
sleep 5
# 启动LXDE桌面环境
startlxde &
# 启动VNC服务器
x11vnc -forever -usepw -create -display :1
