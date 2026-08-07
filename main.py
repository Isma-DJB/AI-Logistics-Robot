from src.brain.robot_brain import RobotBrain


brain = RobotBrain()

camera_data = "box"

result = brain.analyze(camera_data)

print(result["object"])
print(result["action"])