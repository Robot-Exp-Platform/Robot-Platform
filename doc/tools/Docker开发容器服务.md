# Docker

人们群众苦配环境久矣，docker作为一种容器技术，可以将依托封装好的环境邦在一起，仅仅通过传递 Dockerfile 或者 使用 docker image就可以在不同的设备上配置完全相同的环境，这是何其的方便快捷。

## Docker image 方案（推荐）

直接使用我编译好的 image！这太帅了，除非公司把你墙了。

### windows环境中配置 docker 并拉取

- 下载 wsl2

在命令行或者 powershell 中输入

```shell
wsl --install
```

一键搞定。部分公司可能会默认不信任微软，但是微软也提供了解决指导<https://learn.microsoft.com/zh-cn/windows/wsl/troubleshooting>
![图 0](images/%E5%BE%AE%E8%BD%AF%E6%8F%90%E4%BE%9B%E7%9A%84wsl%E5%B8%AE%E5%8A%A9.png)  

当然，如果这也不成功的话也可以弃置不用，还有高手
微软为windows 自带了 Hyper-V 功能，可以绕开 wsl。参考 <https://blog.csdn.net/joeyoj/article/details/136427362>

那什么，微软是这样的，开发人员一键安装就好了，windows 需要考虑的就多了。

- 下载 docker desktop
虽然可以绕开wsl，但是docker desktop 依然是必须的，因为他是docker的官方版本，而且他的图形化界面也是非常好用的。

网上直接搜索 docker desktop 直接下载安装就好。过程中会问你用不用wsl2，结论是如果刚刚是一键下载的就用，如果是 hyper-v 绕开的话就不用

安装好之后他会转圈圈启动，启动之后你电脑的 docker 环境就搭建好了。

- 下载 vscode docker 插件
![图 1](images/vscodeDocker%E6%8F%92%E4%BB%B6.png)  
大概是这样

-拉取镜像
保证 docker desktop 已经启动，随便开一个命令行，输入

```shell
docker pull registry.cn-hangzhou.aliyuncs.com/yixing312/robot_exp:0.1.0
```

![图 2](images/docker%E6%8B%89%E5%8F%96.png)  

一会儿之后在插件中就会显现一个名字巨长的 image
![图 4](images/image%E6%93%8D%E4%BD%9C.png)  
千万记得选第二个 `run interactive`
![图 5](images/contain%E6%93%8D%E4%BD%9C.png)  

这样就好啦

### linux环境中配置 docker 并拉取

命令行输入！
偷懒了 <https://blog.csdn.net/weixin_43755251/article/details/127512751>

## Dockerfile 方案

在 robot_platform 文件夹根目录下有个 Dockerfile 文件，运行

```shell
docker build -t <你喜欢的名字> .
```

千万记得后面有个点

如果网络环境合适的话，就能自己编译一个 inage 出来拉，大概需要 10 分钟左右
