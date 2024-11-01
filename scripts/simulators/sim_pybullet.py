import time
import json
import argparse
import pybullet as p
import pybullet_data

from robot_pybullet.load import load_robot
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
    # 根据参数选择数据来源
    if args.text:
        robot_config = parse_text_input(args.text)
    elif args.file:
        robot_config = load_robots_from_file(args.file)
    else:
        print("Please specify either -t for command line input or -f for file input.")
        return

    # 遍历机器人列表并加载
    robots = []
    for robot_type, robot_name, position in robot_config:
        robots.append(load_robot(robot_type, robot_name, position))

    # 运行仿真主循环
    try:
        while True:
            # 整理机器人状态并发给 Rust
            robot_states = []
            for robot in robots:
                robot_states.append(robot.get_state())
            req.send_state(robot_states)

            # 接收 Rust 发来的命令
            command = json.loads(req.receive_command())
            # print("pybullet: get command", command)
            for robot, cmd in zip(robots, command):
                # print(f"Executing command for {robot}: {cmd}")
                robot.execute_command(cmd)

            # 更新仿真
            p.stepSimulation()
            time.sleep(0.004)
    except KeyboardInterrupt:
        print("Simulation terminated by user.")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
