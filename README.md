# Readme

英文文档待撰写

## Introduction

本项目是一个基于 Rust 的机器人实验平台，在这里，您只需要通过选择已经写好的控制器与规划器（当然，自己编写的也完全可以），就可以快速地搭建起一个机器人的实验环境，而无需担心通讯、节点管理、线程管理等一系列麻烦的事情，只需要专注于您的算法实现。

### why Rust?

实际上我们的工作与 matlab 中 simulink 很是相似，都是期望能够模块化的搭建一个实验平台。但是本系统采用 rust 出于以下几方面的考虑：

1. Rust 是一门系统级别的语言，其性能与 C 语言相当，得益于其零成本抽象，在部分情况下其性能甚至优于C++
2. Rust 在安全性上有着相当好的表现，这对于机器人实验平台来说，运行时的代码安全性直接关乎到实验设备的安全性。
3. Rust 的静态类型系统使得我们几乎可以实现编码完成=运行成功。而无需面对令人焦头烂额的运行时错误和漫长的debug过程。这点想必此前使用 C++/ros 的同学们深有体会。

总而言之我们一方面希望能够像 simulink 一样提供一个模块化的实验平台，另一方面我们也追求最优的运行时性能和代码安全性。Rust 无疑是最好的选择了。

至于 Rust 陡峭的学习曲线，我们在系统搭建的过程中已经尽量完成了其中较为复杂的部分，在具体的控制器和规划器的实现中，实际上更多的还是常见的矩阵乘法等数学运算，而无需担忧所有权、生命周期等令人头大的概念。剩余的这点难度，就当时本系统的小小门槛吧。

### Ros?

我们确实保留了 `ros` 接口，您分别可以用 `-feature ros` 和 `-feature ros2` 启动响应的对外接口（这当然要求您的计算机已经安装配置好了 `ros` 或者 `ros2` 的环境，这通常是在 `ubuntu` 下）。当然，我们也提供了避开 `ros` 的方案（此时您不可以使用rviz等 `ros` 工具，但是仍然可以使用其他的仿真器工具如 `pybullet` 等）以供您在 `windows` 或者其他不想配置 ros 的环境下使用。

### 多机械臂协作？

这当然是可以的，我们的系统希望能够支持多种不同类型的机械臂同时协作，这只需要一个机器人对应的参数文件即可。

同时，我们的系统也可以仿真与真实机器人同步协作，您可以选择您的机器人集群中部分机器人采用不同型号的真实机器人，部分机器人采用不同的仿真器进行仿真。系统将会自动处理来自不同消息源的数据信息，并根据您的要求完成协作任务。

## Quick Start

环境要求：

- rust nightly版本
- python 3.10+
- （可选）ros noetic 或者 ros2

请克隆本项目到本地：

```bash
git clone git@github.com:Robot-Exp-Platform/Robot-Platform.git
```

在项目根目录中检查是否有系统架构文件和任务参数文件，一个直接可用的例子是：

```json
// config.json
{
  "robots": [
    { "name": "panda_1", "robot_type": "panda", "base_pose": { "rotation": [1.0, 0.0, 0.0, 0.0], "translation": [0.0, 0.0, 0.0] } },
    { "name": "panda_2", "robot_type": "panda", "base_pose": { "rotation": [1.0, 0.0, 0.0, 0.0], "translation": [0.0, 1.0, 0.0] } }
  ],
  "sensors": [
    {
      "name": "obstacle_list_1",
      "sensor_type": "obstacle_list",
      "params": [{ "Sphere": { "id": 1, "pose": { "rotation": [1, 0, 0, 0], "translation": [0, 0, 0] }, "params": 0.1 } }]
    }
  ]
}
```

```json
// task.json
[
  {
    "id": 0,
    "rely": [],
    "target": [],
    "nodes": [["bullet", ["panda_1", "panda_2"], ["obstacle_list_1"], { "period": 0.0, "config_path": "./config/config.json" }]],
    "edges": [[1, 0]]
  },
  {
    "id": 1,
    "rely": [],
    "target": [
      { "Transform": [1, { "rotation": [1, 0, 0, 0], "translation": [-1, 2, 1] }, { "rotation": [1, 0, 0, 0], "translation": [0.5, 1, 0.5] }] }
    ],
    "nodes": [["obstacle_releaser", [], ["obstacle_list_1"], { "period": 0.1, "interp": 10 }]],
    "edges": [[0, 1]]
  },
  {
    "id": 2,
    "rely": [0],
    "target": [
      { "Joint": [[0.0124, -0.8838, 0.3749, -2.2172, 0.232, 1.7924, 1.3719], 7, null] },
      { "Joint": [[0.2896, -1.0286, 0.6738, -2.0833, 0.551, 2.1874, 1.0705], 7, null] },
      { "Joint": [[0.0592, -0.3941, 0.4692, -1.6001, 0.1456, 2.0968, 1.201], 7, null] },
      { "Joint": [[0.1, -0.8292, 0.7548, -2.3791, 0.1615, 2.2308, 1.4292], 7, null] },
      {
        "Pose": {
          "rotation": [-0.12855668591466213, -0.15700477626306164, -0.490559104312501, -0.847451735447714],
          "translation": [0.22043052593419135, -0.1830444208109815, -0.778266001811583]
        }
      },
      {
        "Pose": {
          "rotation": [-0.36610076564948413, -0.27304937723042694, -0.07558560120229914, -0.8863978135554177],
          "translation": [-0.035407424860745586, -0.3185646453821069, -0.8141376676593793]
        }
      },
      {
        "Pose": {
          "rotation": [-0.30461916487410634, -0.27677501069715693, -0.28556663744662825, -0.8654793200431868],
          "translation": [0.2707971478614392, -0.2587929961421303, -0.916820013033415]
        }
      },
      {
        "Pose": {
          "rotation": [-0.13546574916388018, -0.2835223923542693, -0.3175409259012247, -0.8946685666854371],
          "translation": [0.1536826534442137, -0.3493401731427769, -0.662014982032004]
        }
      }
    ],
    "nodes": [
      ["cfs", ["panda_1"], ["obstacle_list_1"], { "period": 0.95, "ninterp": 7, "niter": 100, "cost_weight": [0, 10.0, 20.0], "solver": "osqp" }],
      ["interp", ["panda_1"], ["obstacle_list_1"], { "period": 0.1, "interp_fn": "lerp", "ninter": 25 }],
      ["position", ["panda_1"], [], { "period": 0.004 }]
    ],
    "edges": [
      [0, 1],
      [1, 2],
      [2, 3],
      [3, 0]
    ]
  },
  {
    "id": 3,
    "rely": [0],
    "target": [
      { "Joint": [[0.0124, -0.8838, 0.3749, -2.2172, 0.232, 1.7924, 1.3719], 7, null] },
      { "Joint": [[0.2896, -1.0286, 0.6738, -2.0833, 0.551, 2.1874, 1.0705], 7, null] },
      { "Joint": [[0.0592, -0.3941, 0.4692, -1.6001, 0.1456, 2.0968, 1.201], 7, null] },
      { "Joint": [[0.1, -0.8292, 0.7548, -2.3791, 0.1615, 2.2308, 1.4292], 7, null] },
      { "Joint": [[0.0124, -0.8838, 0.3749, -2.2172, 0.232, 1.7924, 1.3719], 7, null] },
      { "Joint": [[0.2896, -1.0286, 0.6738, -2.0833, 0.551, 2.1874, 1.0705], 7, null] },
      { "Joint": [[0.0592, -0.3941, 0.4692, -1.6001, 0.1456, 2.0968, 1.201], 7, null] },
      { "Joint": [[0.1, -0.8292, 0.7548, -2.3791, 0.1615, 2.2308, 1.4292], 7, null] }
    ],
    "nodes": [
      ["cfs", ["panda_2"], ["obstacle_list_1"], { "period": 0.95, "ninterp": 7, "niter": 10, "cost_weight": [0, 10.0, 20.0], "solver": "osqp" }],
      ["interp", ["panda_2"], ["obstacle_list_1"], { "period": 0.1, "interp_fn": "lerp", "ninter": 25 }],
      ["position", ["panda_2"], [], { "period": 0.004 }]
    ],
    "edges": [
      [0, 1],
      [1, 2],
      [2, 3],
      [3, 0]
    ]
  }
]
```

最简样例中采用了 Franka Emika Panda 机械臂，使用了 pid 控制器和 linear 规划器，使用 bullet 仿真器进行仿真。您可以根据自己的需求修改这两个文件。

在根目录中运行：

```bash
cargo build
cargo run --release
```

或者仅仅运行：

```bash
cargo build
cargo run
```

您就可以看到机械臂在仿真器中运动了。同时命令行中会输出机械臂的状态信息和规划控制信息。

## 不同节点的标准启动配置

```json
[
  ["cfs", ["panda_1"], ["obstacle_releaser_1"], { "period": 0.8, "ninterp": 7, "niter": 6, "cost_weight": [0, 10.0, 20.0], "solver": "osqp" }],
  [ 
    "cfs_end_pose", ["panda_1"], ["obstacle_releaser_1"],
    { "period": 0.95, "ninterp": 7, "niter": 10, "cost_weight": [0, 10.0, 20.0], "solver": "osqp" }
  ],
  ["interp", ["panda_1"], ["obstacle_releaser_1"], { "period": 0.1, "interp_fn": "lerp", "ninter": 25 }],

  [
    "impedence_diag", ["panda_1"], [],
    {
      "period": 0.004,
      "k": [[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], 7, null],
      "b": [[50.0, 50.0, 50.0, 20.0, 20.0, 20.0, 10], 7, null],
      "m": [[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 7, null]
    }
  ],
  [
    "pid_diag", ["panda_1"], [],
    {
      "period": 0.004,
      "k": [[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], 7, null],
      "b": [[50.0, 50.0, 50.0, 20.0, 20.0, 20.0, 10], 7, null],
      "m": [[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 7, null]
    }
  ],
  ["position", ["panda_1"], [], { "period": 0.05 }],
  ["force", ["panda_1"], [], { "period": 0.004 }]

  ["bullet", ["panda_1"], ["obstacle_releaser_1"], { "period": 0.0 }]
]
```

## Documentation

暂且还不完备，02 阶段完成后会整理文档

## Contribution Guidelines

![System Architecture](./images/system%20architecture.png)

### 代码风格

在本系统中诸多代码需各自遵循以下代码规范：

- rust rustfmt 需要安装 rust-analyzer 插件
- python pylint 需要安装 pylint 插件
- markdown markdownlint 需要安装 markdownlint 插件

推荐的报错插件和注释插件为：

- error lens
- better comments

其中部分约束并非强制要求，建议在 ./vscode/settings.json 中添加以下配置：

```json
//*? 以下是对 markdownlint 的配置
  "markdownlint.config": {
    "default": true,
    "MD025": false,
    "MD033": false,
    "MD024": false
  },
  "pylint.args": ["--disable=C0116,I1101", "--max-line-length=120"],
  "flake8.args": ["--max-line-length=120"]
```

### 系统结构

整个实验实际上是基于一个实验对象： Exp

```rust
pub struct Exp {
    // Exp 是一个森林状的结构，其中的包含 robot tree, controller tree, planner tree 等等树状结构的根节点，通过管理 exp 实现管理整个结构的目的
    pub thread_manage: ThreadManage,
    pub task_manage: TaskManager,

    pub robot_exp: Arc<RwLock<dyn robot::Robot>>,
    pub planner_exp: Arc<Mutex<dyn planner::Planner>>,
    pub controller_exp: Arc<Mutex<dyn controller::Controller>>,
    pub simulator_exp: Arc<Mutex<dyn simulator::Simulator>>,
}
```

其中包含一个线程管理器、一个任务管理器和一些树状结构的根节点。

对于单机器人结构，其结构如流程图所是：

```mermaid
graph TD
    T[task_manager] -->|Target| A[planner]
    A -->|Track| B[controller]
    B -->|ControlCommand| C[simulator]
    C --> D[robot]
```

当多机器人时，robot 树中每个叶节点对应一个机器人，每个机器人都有其对应的 planner, controller, simulator。所以这几棵子树的形状保持一致。

每个 节点，包含 planner, controller, simulator, 都实现了 RosThread 特征，线程管理器将管理所有的节点线程。

```rust
pub trait ROSThread: Send + Sync {
    fn init(&mut self) {}
    fn start(&mut self) {}
    fn update(&mut self) {}
    fn get_period(&self) -> Duration {
        Duration::from_secs(0)
    }
}
```

```rust
pub struct ThreadManage {
    threads: Vec<thread::JoinHandle<()>>,
    pub condvar: Arc<(AtomicBool, Condvar, Mutex<()>)>,
}

impl ThreadManage {
  pub fn add_thread(&mut self, node: Arc<Mutex<dyn ROSThread>>) {
        let condvar = self.condvar.clone();
        let node = node.clone();
        let thread = thread::spawn(move || {
            let mut node_lock = node.lock().unwrap();
            node_lock.init();
            drop(node_lock);
            let (flag, cvar, lock) = &*condvar;
            loop {
                let mut locked = lock.lock().unwrap();
                while !flag.load(Ordering::SeqCst) {
                    locked = cvar.wait(locked).unwrap();
                }
                drop(locked);

                let mut node_lock = node.lock().unwrap();
                node_lock.start();
                let period = node_lock.get_period();
                while flag.load(Ordering::SeqCst) {
                    let start_time = Instant::now();

                    node_lock.update();

                    let elapsed_time = start_time.elapsed();
                    if period > elapsed_time {
                        thread::sleep(period - elapsed_time);
                    }
                }
                drop(node_lock);
            }
        });
        self.threads.push(thread);
    }

    pub fn start_all(&self) {
        let (flag, cvar, _) = &*self.condvar;
        flag.store(true, Ordering::SeqCst);
        cvar.notify_all();
    }

    pub fn stop_all(&self) {
        let (flag, _, _) = &*self.condvar;
        flag.store(false, Ordering::SeqCst);
    }
}
```

这里使用了 condvar 作为线程开关， 控制所有线程的同时启动和停止。
