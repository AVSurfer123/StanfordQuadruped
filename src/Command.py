import numpy as np


class Command:
    """Stores movement command
    """

    def __init__(self, height):
        self.horizontal_velocity = np.array([0, 0])
        self.yaw_rate = 0.0
        self.height = height
        self.yaw = 0
        self.pitch = 0.0
        self.roll = 0.0

        self.walk_event = False
        self.trot_event = False
        self.stand_event = False
        self.activate_event = False
        self.deactivate_event = False
        self.home_event = False
        self.auton_mode = False

    def __str__(self):
        return "vx: {} vy: {} wz: {} height: {} yaw: {} pitch: {} roll: {} trot_event: {} home event: {} auton mode: {}".format(
            self.horizontal_velocity[0],
            self.horizontal_velocity[1],
            self.yaw_rate,
            self.height,
            self.yaw,
            self.pitch,
            self.roll,
            self.trot_event,
            self.activate_event,
            self.home_event,
            self.auton_mode
        )
