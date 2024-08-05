# 00-1-2 RosThread

看上去是在写 ros 实际上和并不一定和 ros 有关 ， 只是标准节点管理的方法罢了。

## 相关关键词

> thread node

## 关于线程管理

实际上在系统运行过程中会有相当多的线程在同步运行，我们称一个标准的运行线程为一个节点(node)，并且为这个节点实现了特征 `RosThread`。

```rust
pub trait ROSThread: Send + Sync {
    fn init(&mut self) {}
    fn start(&mut self) {}
    fn update(&mut self) {}
    fn is_running(&self) -> bool { true }
}
```

里面一共就四个方法，分别是 `init` `start` `update` `is_running`，前三个方法分别对应节点的初始化、启动和更新。在标准节点中其运行流程如下：

```rust
|node: dyn RosThread|{
    node.init(); // 整个系统初始化时运行的方法

    loop {
        // 运行一些对节点参数的修改，如修改 pid 的 Kp 值、修改 planner 的插值数等

        node.start(); //任务启动时运行的方法
        while node.is_running() {
            node.update(); // 节点以超高频率更新的方法
        }
    }
}
```

其中 is_running 可以被TheadManager 直接全局修改，以实现全局节点同时开始和停止的效果。

以 pid 为例子，我们可以实现一个 pid 节点如下：

```rust
impl RosThread for Pid {
    fn init(&mut self) {
        // 启动通讯节点
    }

    fn start(&mut self) {
        // 检查和更新参数等一些操作
    }

    fn update(&mut self) {
        // 更新 pid 的输出
        // y = Kp * e + Ki * ∫e + Kd * de/dt
    }
}
```

就是这样喵
