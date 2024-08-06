import pybullet as p
import pybullet_data

def pubullet_init(USE_GUI):

    "pybullet初始化"

    if USE_GUI:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    # 渲染逻辑
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    # 设置 PyBullet 的数据路径
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # 设置重力参数
    p.setGravity(0, 0, -9.81)

    # 设置模拟器步进
    p.setRealTimeSimulation(0)

    # 加载地板模型
    planeId = p.loadURDF("plane.urdf")

    return planeId

class Joint:
    def __init__(self, robot_id, joint_index) -> None:
        # 关节基本信息
        self.robot_id = robot_id
        self.joint_index =joint_index
        self.info = p.getJointInfo(robot_id, joint_index)
        self.name = self.info[1].decode('utf-8')
        self.lower_limit = self.info[8]
        self.upper_limit = self.info[9]
        # 关节状态参数
        self.state = p.getJointState(robot_id, joint_index)
        self.position = self.state[0] 
        self.velocity = self.state[1]
    
    def update_state(self):
        # 更新关节状态参数
        self.state = p.getJointState(self.robot_id, self.joint_index)
        self.position = self.state[0]
        self.velocity = self.state[1]

    def reset_joint_state(self):
        p.resetJointState(self.robot_id, self.joint_index, 0)
        self.update_state()

    def get_state(self):
        # 返回位置与速度
        self.update_state()
        return self.position, self.velocity
    
    def set_velocity(self, velocity, force = 500):
        p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=self.joint_index,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=velocity,
                force=force  # 设置最大力矩或推力
            )
        self.update_state()


class Panda:
    def __init__(self, base_position = [0, 0 ,0]) -> None:
        self.panda_path = "franka_panda/panda.urdf"
        self.base_position = base_position
        self.id = p.loadURDF(self.panda_path, useFixedBase=True, basePosition = base_position)
        self.joints_num = 7
        self.control_joint = [] # 存储控制的关节的列表
        for i in range(0, self.joints_num):
            self.control_joint.append(Joint(self.id, i))
        self.reset_panda_state()
    
    def reset_panda_state(self):
        for joint in self.control_joint:
            joint.reset_joint_state()
    
    def print_panda_info(self):
        print("*"*50)
        print("Robot Id: ", self.id)
        print("Robot Position: ", self.base_position)
        print("Robot joints num: ", self.joints_num)
        for joint in self.control_joint:
            print(f"Joint {joint.joint_index} ({joint.name}): Limits = ({joint.lower_limit}, {joint.upper_limit})")
        print("*"*50)

    def get_joint_state(self, joint_index):
        return self.control_joint[joint_index].get_state()
    
    def get_joints_state(self):
        joint_positions = []
        joint_velocities = []
        for joint in self.control_joint:
            position, velocity = joint.get_state()
            joint_positions.append(position)
            joint_velocities.append(velocity)
        return joint_positions, joint_velocities

    def set_joint_velocity(self, joint_index, velocity):
        self.control_joint[joint_index].set_velocity(velocity)

    def set_joints_velocity(self, velocities):
        for i in range(0, self.joints_num):
            self.control_joint[i].set_velocity(velocities[i])

    def print_panda_state(self):
        joint_positions, joint_velocities = self.get_joints_state()
        for i in range(0, 7):
            print(f"Joint {i}: Angle = {joint_positions[i]:.2f}, Velocity = {joint_velocities[i]:.2f}")
