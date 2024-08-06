import pybullet as p
import time
from robot import pubullet_init, Panda

USE_GUI = True
NOT_USE_GUI = False

if __name__ =="__main__":
    pubullet_init(NOT_USE_GUI)
    
    panda1 = Panda([100,0,0])
    panda2 = Panda([-100,0,0])
    panda1.print_panda_info()
    panda2.print_panda_info()
    target_velocities1 = [0.5, 1.0, 1.5, -1.0, -0.5, 0.7, -0.3]
    target_velocities2 = [-0.5, -1.0, -1.5, 1.0, 0.5, -0.7, 0.3]
    panda1.set_joints_velocity(target_velocities1)
    panda2.set_joints_velocity(target_velocities2)
    for _ in range(500):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
    panda1.print_panda_state()
    print("-"*20)
    panda2.print_panda_state()
    print("*"*20)