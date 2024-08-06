import pybullet as p
import time
from robot import pubullet_init, Panda
from zmq2rust import ZMQ_REQ

USE_GUI = True
NOT_USE_GUI = False

ZMQ_HOST = "tcp://localhost:5555"

if __name__ =="__main__":
    pubullet_init(USE_GUI)

    Req = ZMQ_REQ(ZMQ_HOST)
    
    panda1 = Panda([0.5,0,0])
    panda2 = Panda([-0.5,0,0])
    panda1.print_panda_info()
    panda1.print_panda_state()
    # target_velocities1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # target_velocities2 = [-0.5, -1.0, -1.5, 1.0, 0.5, -0.7, 0.3]
    # panda1.set_joints_velocity(target_velocities1)
    # panda2.set_joints_velocity(target_velocities2)
    for _ in range(5000):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
    # joint_positions, joint_velocities = panda1.get_joints_state()
    # Req.send_array(joint_positions + joint_velocities)
    # print(Req.receive_array())
