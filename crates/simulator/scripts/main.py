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
    period = 0
    velocities = [0, 0, 0, 0, 0, 0, 0]
    print("+" * 20)
    panda1.print_panda_info()
    print("+" * 20)
    while True:
        print("*" * 20)
        panda1.print_panda_state()
        print("*" * 20)
        joint_positions, joint_velocities = panda1.get_joints_state()
        Req.send_array(joint_positions + joint_velocities)
        period, velocities = Req.receive_command()
        print("period: ", period)
        print("joint velocities: ", velocities)
        panda1.set_joints_velocity(velocities)
        for i in range(int(period * 240)):
            p.stepSimulation()
