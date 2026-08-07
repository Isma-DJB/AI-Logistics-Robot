from src.brain.robot_brain import RobotBrain


brain = RobotBrain()

camera_data = "person"

result = brain.analyze(camera_data)

print(result)