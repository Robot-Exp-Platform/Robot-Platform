import pybullet as p
import pybullet_data
import time

USE_GUI = True
if USE_GUI:
    p.connect(p.GUI)
else:
    p.connect(p.DIRECT)

# 设置 PyBullet 的数据路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 加载 Panda 机器人 URDF 文件
panda_path = "franka_panda/panda.urdf"
robot_id = p.loadURDF(panda_path, useFixedBase=True)

# 关节数量，Panda 机器人有 7 个自由度
num_joints = p.getNumJoints(robot_id)

# 模拟器步进
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# 设置初始位置和控制目标
initial_joint_positions = [0.0] * num_joints
for joint_index in range(num_joints):
    p.resetJointState(robot_id, joint_index, initial_joint_positions[joint_index])

for joint_index in range(0, 7):
    joint_info = p.getJointInfo(robot_id, joint_index)
    joint_name = joint_info[1].decode('utf-8')
    lower_limit, upper_limit = joint_info[8], joint_info[9]
    print(f"Joint {joint_index} ({joint_name}): Limits = ({lower_limit}, {upper_limit})")


# 设置关节的速度
target_velocity = -1.0  # rad/s for revolute joints or m/s for prismatic joints
for joint_index in range(0, 7):
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=joint_index,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=target_velocity,
        force=100.0  # 设置最大力矩或推力
    )

# 模拟几秒以便观察
for _ in range(2400):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

# 获取并打印关节角度与速度
joint_angles = []
joint_velocities = []

# 前7个为需要控制的关节
for joint_index in range(0, 7):
    joint_info = p.getJointState(robot_id, joint_index)
    joint_angle = joint_info[0]
    joint_velocity = joint_info[1]
    
    joint_angles.append(joint_angle)
    joint_velocities.append(joint_velocity)

    print(f"Joint {joint_index}: Angle = {joint_angle:.2f}, Velocity = {joint_velocity:.2f}")

# 断开连接
p.disconnect()