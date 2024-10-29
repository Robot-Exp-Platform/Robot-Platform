from robot_pybullet.franka_panda import Panda


def load_robot(robot_type, _, position):
    match robot_type:
        case "panda":
            return Panda(position)
        case _:
            raise ValueError(f"Unknown robot type: '{robot_type}'.")
