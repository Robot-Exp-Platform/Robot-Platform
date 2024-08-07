import pybullet as p
import time
from robot import pubullet_init, Panda
from zmq2rust import ZMQ_REQ

USE_GUI = True
NOT_USE_GUI = False

ZMQ_HOST = "tcp://localhost:5555"

if __name__ =="__main__":
    pubullet_init(NOT_USE_GUI)

    Req = ZMQ_REQ(ZMQ_HOST)
    
    panda1 = Panda([0.5,0,0])
    panda2 = Panda([-0.5,0,0])
    while True:
        p.stepSimulation()
        joint_positions, joint_velocities = panda1.get_joints_state()
        Req.send_array(joint_positions + joint_velocities)
        period, velocities = Req.receive_command()
        print("period: ", period)
        print("joint velocities: ", velocities)
