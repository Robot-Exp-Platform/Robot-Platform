# Docker 可视化前端

Docker 可视化的核心在于赋予这个命令行系统一个“显示器”，然后将这个显示器接在我们的电脑中。

## VNC 方案

> 0.3.5 、 0.3.6 版本支持该方案

首先拉取

```shell
docker pull registry.cn-hangzhou.aliyuncs.com/yixing312/robot_exp:0.3.6
```

然后在运行过程中需要转发 5900 端口

```shell
docker run -it -p 5900:5900 registry.cn-hangzhou.aliyuncs.com/yixing312/robot_exp:0.3.6
```

然后就可以在 vncviewer 中输入 `localhost:5900` 进行连接，就可以获得一个前端桌面了。
其中 vncviewer 是一类软件的统称，是能够查看 vnc 服务器的软件，gpt 推荐 TightVNC Viewer，我用的是 RealVNC Viewer。自行选择就好，效果大概如图所示
![图 0](images/Realvnc%20viewer%20%E7%A4%BA%E4%BE%8B.png)  

建议平常写代码还是直接用 “附加 vscode”
