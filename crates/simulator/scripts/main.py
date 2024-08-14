import pybullet as p
from robot import pybullet_init, Panda
from zmq2rust import ZMQ_REQ

USE_GUI = True
NOT_USE_GUI = False

ZMQ_HOST = "tcp://localhost:5555"

if __name__ == "__main__":
    pybullet_init(NOT_USE_GUI)

    Req = ZMQ_REQ(ZMQ_HOST)

    panda1 = Panda([0.5, 0, 0])
    period = 0.05
    velocities = [0, 0, 0, 0, 0, 0, 0]
    print("+" * 20)
    panda1.print_panda_info()
    print("+" * 20)
    while True:
        print("*" * 20)
        panda1.print_panda_state()
        print("*" * 20)
        joint_positions, joint_velocities = panda1.get_joints_state()
        Req.send_state("JointVelocity", [joint_positions, joint_velocities])
        cmd = Req.receive_command()
        print("recieve command !")
        print(cmd)
        match list(cmd.keys())[0]:
            case "Joint":
                period = 0.05
                pos = cmd["Joint"]
                panda1.set_joints_position(pos)
            case "JointWithPeriod":
                period = cmd["JointWithPeriod"][0]
                pos = cmd["JointWithPeriod"][1]
                panda1.set_joints_position(pos)
            case "Tau":
                period = 0.05
                torque = cmd["Tau"]
                panda1.set_joints_torque(torque)
            case "TauWithPeriod":
                period = cmd["TauWithPeriod"][0]
                torque = cmd["TauWithPeriod"][1]
                panda1.set_joints_torque(torque)

        for i in range(int(period * 240)):
            p.stepSimulation()