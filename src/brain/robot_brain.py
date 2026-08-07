class RobotBrain:
    def analyze(self, data):
        if data == "box":
            return {
                "object": "box",
                "action": "pick_up"
            }

        elif data == "pallet":
            return {
                "object": "pallet",
                "action": "move_around"
            }

        elif data == "person":
            return {
                "object": "person",
                "action": "stop"
            }

        else:
            return {
                "object": "unknown",
                "action": "investigate"
            }