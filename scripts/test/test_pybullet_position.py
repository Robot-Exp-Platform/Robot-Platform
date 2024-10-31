import time
import math  # 使用 math 模块的 sin 和 cos 函数
import pybullet as p
import pybullet_data

# 初始化 PyBullet 仿真
physics_client = p.connect(p.GUI)  # GUI 模式
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 设置 URDF 路径
p.setGravity(0, 0, -9.8)

# 加载 Panda 机器人和地面
plane_id = p.loadURDF("plane.urdf")
start_position = [0, 0, 0]  # 起始位置
robot_id = p.loadURDF("franka_panda/panda.urdf", start_position, useFixedBase=True)

# 获取 Panda 机器人的关节信息
num_joints = p.getNumJoints(robot_id)
joint_indices = range(num_joints)

# 设置关节的目标位置
target_positions = [0] * num_joints  # 所有关节的目标位置初始为 0

# 运行仿真主循环
try:
    while True:
        # 修改目标位置 (例如摆动关节 0 和关节 2)
        target_positions[0] = 0.5 * math.sin(time.time())  # 随时间变化的关节角度
        target_positions[2] = 0.5 * math.cos(time.time())  # 随时间变化的关节角度

        # 应用位置控制到每个关节
        for i, target_position in enumerate(target_positions):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_position,
                force=500,  # 设置一个合理的控制力
            )

        # 步进仿真
        p.stepSimulation()
        time.sleep(1.0 / 240.0)  # 控制仿真时间步长

except KeyboardInterrupt:
    print("Simulation terminated by user.")

finally:
    # 断开仿真连接
    p.disconnect()
