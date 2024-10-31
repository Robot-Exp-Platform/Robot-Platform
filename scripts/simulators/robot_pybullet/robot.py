import pybullet as p


class Joint:
    """使用 pybullet 的 Joint 类，用于控制机器人关节。"""

    def __init__(self, robot_id, joint_index) -> None:
        # 关节基本信息
        self.robot_id = robot_id
        self.joint_index = joint_index
        self.info = p.getJointInfo(robot_id, joint_index)
        self.name = self.info[1].decode("utf-8")
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

    def reset_joint_state(self, position=0):
        p.resetJointState(self.robot_id, self.joint_index, position)
        self.update_state()

    def get_state(self):
        # 返回位置与速度
        self.update_state()
        return self.position, self.velocity

    def set_position(self, position, force=2000):
        """位控方案

        Args:
            position (f64): 关节角度
            force (int, optional): _description_. Defaults to 500.
        """
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=self.joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=position,
            force=force,  # 设置最大力矩或推力
        )
        self.update_state()

    # 速度控制方案
    def set_velocity(self, velocity, force=500):
        """速度控制方案

        Args:
            velocity (f64): 关节速度
            force (int, optional): _description_. Defaults to 500.
        """
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=self.joint_index,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=velocity,
            force=force,  # 设置最大力矩或推力
        )
        self.update_state()

    # 力矩控制方案
    def set_torque(self, torque):
        """力控方案

        Args:
            torque (float): 力矩值
        """
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=self.joint_index,
            controlMode=p.TORQUE_CONTROL,
            force=torque,
        )


class Robot:
    """机器人基类，用于实现机器人仿真过程中的基本逻辑"""

    def __init__(self, base_pose=None):
        self.base_pose = base_pose if base_pose is not None else [0, 0, 0]

        self.njoint = 0
        self.default_q = []
        self.control_joint: list[Joint] = []

    def reset(self):
        for joint, position in zip(self.control_joint, self.default_q):
            joint.reset_joint_state(position)

    def get_joint_state(self, joint_index):
        return self.control_joint[joint_index].get_state()

    def get_state(self):
        joint_positions = []
        joint_velocities = []
        for joint in self.control_joint:
            position, velocity = joint.get_state()
            joint_positions.append(position)
            joint_velocities.append(velocity)
        return {"JointVel": [joint_positions, joint_velocities]}

    def set_joint_position(self, joint_index, position):
        self.control_joint[joint_index].set_position(position)

    def set_joints_position(self, positions):
        for joint, position in zip(self.control_joint, positions):
            joint.set_position(position)

    def set_joint_velocity(self, joint_index, velocity):
        self.control_joint[joint_index].set_velocity(velocity)

    def set_joints_velocity(self, velocities):
        for joint, velocity in zip(self.control_joint, velocities):
            joint.set_velocity(velocity)

    def set_joint_torque(self, joint_index, torque):
        self.control_joint[joint_index].set_torque(torque)

    def set_joints_torque(self, torques):
        for joint, torque in zip(self.control_joint, torques):
            joint.set_torque(torque)

    def execute_command(self, cmd):
        """执行命令

        Args:
            cmd (dict): 命令
        """
        if "Joint" in cmd:
            self.set_joints_position(cmd["Joint"][0])
        elif "JointWithPeriod" in cmd:
            self.set_joints_position(cmd["JointWithPeriod"][1])
        elif "Tau" in cmd:
            self.set_joints_torque(cmd["Tau"][0])
        elif "TauWithPeriod" in cmd:
            self.set_joints_torque(cmd["TauWithPeriod"][1])
