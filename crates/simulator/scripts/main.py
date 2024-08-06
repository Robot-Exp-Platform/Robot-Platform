import pybullet as p
import time
from robot import pubullet_init, Panda

USE_GUI = True
NOT_USE_GUI = False

if __name__ =="__main__":
    pubullet_init(NOT_USE_GUI)
    
    panda = Panda()

    target_velocities = [0.5, 1.0, 1.5, -1.0, -0.5, 0.7, -0.3]
    panda.set_joint_velocity(target_velocities)
    for _ in range(500):
        p.stepSimulation()
        # time.sleep(1.0 / 240.0)
        time.sleep(1.0 / 240.0)

    panda.print_panda_state()