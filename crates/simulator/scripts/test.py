import pybullet as p
import pybullet_data
import time

USE_GUI = False
if USE_GUI:
    p.connect(p.GUI)
else:
    p.connect(p.DIRECT)

# 设置 PyBullet 的数据路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 加载 Panda 机器人 URDF 文件
panda_path = "franka_panda/panda.urdf"
robot_id = p.loadURDF(panda_path, useFixedBase=True)

# 过滤出可控关节（通常类型为REVOLUTE或PRISMATIC）
controlled_joints = []
for joint_index in range(p.getNumJoints(robot_id)):
    joint_info = p.getJointInfo(robot_id, joint_index)
    joint_type = joint_info[2]
    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:  # 旋转或滑动关节
        controlled_joints.append(joint_index)

# 模拟器步进
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# 设置初始位置和控制目标
initial_joint_positions = [0.0] * len(controlled_joints)
for idx, joint_index in enumerate(controlled_joints):
    p.resetJointState(robot_id, joint_index, initial_joint_positions[idx])

# 模拟几秒以便观察
for _ in range(240):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

# 获取并打印关节角度与速度
joint_angles = []
joint_velocities = []

for idx, joint_index in enumerate(controlled_joints):
    joint_info = p.getJointState(robot_id, joint_index)
    joint_angle = joint_info[0]
    joint_velocity = joint_info[1]
    
    joint_angles.append(joint_angle)
    joint_velocities.append(joint_velocity)

    print(f"Joint {joint_index}: Angle = {joint_angle:.2f}, Velocity = {joint_velocity:.2f}")

# 断开连接
p.disconnect()