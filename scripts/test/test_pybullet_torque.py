import pybullet as p
import pybullet_data
import time
import math

# 初始化 PyBullet 仿真环境
physics_client = p.connect(p.GUI)  # GUI 模式
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 设置 URDF 路径
p.setGravity(0, 0, -9.8)

# 加载 Panda 机器人和地面
plane_id = p.loadURDF("plane.urdf")
start_position = [0, 0, 0]
robot_id = p.loadURDF("franka_panda/panda.urdf", start_position, useFixedBase=True)

# 获取 Panda 机器人的关节信息
num_joints = p.getNumJoints(robot_id)
joint_indices = list(range(num_joints))

# # 禁用位置控制模式，启用力矩控制
# for joint_index in joint_indices:
#     p.setJointMotorControl2(
#         bodyIndex=robot_id,
#         jointIndex=joint_index,
#         controlMode=p.VELOCITY_CONTROL,
#         force=0,
#     )

# 运行仿真主循环
try:
    while True:
        # 设置力矩控制：例如，使关节0到6周期性地施加正弦波力矩
        torques = [0.5 * math.sin(time.time() + i) for i in range(7)]

        # 应用力矩到每个关节
        for joint_index, torque in zip(joint_indices[:7], torques):  # 忽略非驱动关节
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint_index,
                controlMode=p.VELOCITY_CONTROL,
                force=torque,
            )

        # 步进仿真
        p.stepSimulation()
        time.sleep(1.0 / 240.0)  # 仿真时间步长

except KeyboardInterrupt:
    print("Simulation terminated by user.")

finally:
    # 断开连接
    p.disconnect()
