import time
import json
import argparse
import pybullet as p
import pybullet_data

from robot_pybullet.load import load_robot, load_obstacle
from utilities.zmq2rust import ZmqReq


# 创建命令行参数解析器
def parse_args():
    parser = argparse.ArgumentParser(description="PyBullet Robot Simulation")
    parser.add_argument(
        "-t",
        "--text",
        type=str,
        nargs="+",
        help="List of robot names with format 'type_id', e.g., 'panda_1 ur_4'",
    )
    parser.add_argument(
        "-f",
        "--file",
        type=str,
        help="Path to a JSON file containing the list of robots with their type and name.",
    )
    return parser.parse_args()


# 从文件中读取机器人配置
def load_robots_from_file(file_path):
    with open(file_path, "r", encoding="utf8") as file:
        data = json.load(file)
    return [
        (robot["robot_type"], robot["name"], robot["base_pose"]["translation"])
        for robot in data.get("robots", [])
    ]


def load_obstacles_from_file(file_path):
    with open(file_path, "r", encoding="utf8") as file:
        data = json.load(file)
    return [obstacle for obstacle in data.get("obstacles", [])]


# 从命令行输入中解析机器人类型和名称
def parse_text_input(robot_list):
    robots = []
    for robot in robot_list:
        try:
            robot_type, robot_id = robot.split("_")
            robot_name = f"{robot_type}_{robot_id}"
            robots.append((robot_type, robot_name))
        except ValueError:
            print(f"Invalid format for robot entry: '{robot}'. Use 'type_id'.")
    return robots


def main():
    args = parse_args()

    # 建立 zmq 通讯
    req = ZmqReq("tcp://localhost:5555")

    # 初始化 PyBullet 仿真
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    # 加载 Panda 机器人和地面
    p.loadURDF("plane.urdf")
    robot_config = []
    obstacle_config = []
    # 根据参数选择数据来源
    if args.text:
        robot_config = parse_text_input(args.text)
    elif args.file:
        robot_config = load_robots_from_file(args.file)
        obstacle_config = load_obstacles_from_file(args.file)
    else:
        print("Please specify either -t for command line input or -f for file input.")
        return

    # 遍历机器人列表并加载
    robots = []
    for robot_type, robot_name, position in robot_config:
        robots.append(load_robot(robot_type, robot_name, position))

    # robot_table 一张映射表，用于输入障碍物 id 获取在 pybullet 中的障碍物对象
    obstacle_table = {}

    # 准备加载障碍物，但是配置文件只包含静态障碍物信息，动态障碍物信息需要等待 Rust 段发送过来或者其他方法获取
    obstacles = []
    for config in obstacle_config:
        obstacles.append(load_obstacle(config))

    # 运行仿真主循环
    try:
        while True:
            # 整理机器人状态并发给 Rust
            robot_states = []
            for robot in robots:
                robot_states.append(robot.get_state())
            req.send_state(robot_states)

            # 接收 Rust 发来的命令
            receive = req.receive()
            # print("pybullet: receive: ", receive)
            for robot, cmd in zip(robots, receive["command"]):
                # print(f"Executing command for {robot}: {cmd}")
                robot.execute_command(cmd)

            # 检查指令中的障碍物，如果是新增的障碍物则加载并放入 obstacles 列表， 如果是已有的障碍物则更新位置， 如果是原有的障碍物但是没有传回相关数据则不处理
            for obstacle_data in receive["obstacles"]:
                for _, obstacle in obstacle_data.items():
                    obstacle_id = obstacle["id"]
                    # 如果障碍物不在 obstacle_table 中，则加载新的障碍物
                    if obstacle_id not in obstacle_table:
                        # 加载新的障碍物
                        new_obstacle = load_obstacle(obstacle_data)
                        obstacles.append(new_obstacle)
                        obstacle_table[obstacle_id] = new_obstacle

                    # 更新障碍物位置
                    p.resetBasePositionAndOrientation(
                        obstacle_table[obstacle_id],
                        obstacle["pose"]["translation"],
                        obstacle["pose"]["rotation"],
                    )

            # 更新仿真
            p.stepSimulation()
            time.sleep(0.004)
    except KeyboardInterrupt:
        print("Simulation terminated by user.")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
