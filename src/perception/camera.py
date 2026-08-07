import cv2


class RobotCamera:

    def __init__(self):
        self.camera = cv2.VideoCapture(0)

    def check_camera(self):
        if self.camera.isOpened():
            return "Camera ready"
        else:
            return "Camera not detected"

    def release(self):
        self.camera.release()


if __name__ == "__main__":
    cam = RobotCamera()

    print(cam.check_camera())

    cam.release()