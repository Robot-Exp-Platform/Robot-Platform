pose = {"Pose": {"rotation": [0.0, 0.0, 0.0, 1.0], "translation": [0.0, 0.0, 0.0]}}
joint = {"Joint": [0.0, 0.0, 0.0]}
velocity = {"Velocity": [0.0, 0.0, 0.0]}
acceleration = {"Acceleration": [0.0, 0.0, 0.0]}
joint_velocity = {"JointVelocity": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]}
joint_velocity_acceleration = {
    "JointVelocityAcceleration": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
}

print("Pose: ", pose)
print("Joint: ", joint)
print("Velocity: ", velocity)
print("Acceleration: ", acceleration)
print("JointVelocity: ", joint_velocity)
