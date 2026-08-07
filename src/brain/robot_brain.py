class RobotBrain:
    def analyze(self, data):
        if data == "box":
            return "Action: pick up the box"

        elif data == "pallet":
            return "Action: move around the pallet"

        elif data == "person":
            return "Action: stop"

        else:
            return "Action: investigate"