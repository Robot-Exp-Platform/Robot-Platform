from robot_pybullet.franka_panda import Panda
import pybullet as p


def load_robot(robot_type, _, position):
    match robot_type:
        case "panda":
            return Panda(position)
        case _:
            raise ValueError(f"Unknown robot type: '{robot_type}'.")


def load_obstacle(obstacle_config: dict):
    if "Sphere" in obstacle_config:
        center = obstacle_config["Sphere"]["center"]
        radius = obstacle_config["Sphere"]["radius"]
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=radius)
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_SPHERE, radius=radius
        )
        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=center,
        )
        return obstacle_id

    else:
        return None
