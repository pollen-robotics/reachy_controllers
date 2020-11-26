import numpy as np

from typing import List
from threading import Thread, Event

from pypot.dynamixel import DxlIO


from .robot_hardware_interface import RobotHardwareABC

reachy_config = {
    '/dev/ttyACM0': [
        ('l_shoulder_pitch', 20, 90.0, True),
        ('l_shoulder_roll', 21, -90.0, False),
        ('l_arm_yaw', 22, 0.0, False),
        ('l_elbow_pitch', 23, 0.0, False),
        ('l_forearm_yaw', 24, 0.0, False),
        ('l_wrist_pitch', 25, 0.0, False),
        ('l_wrist_roll', 26, 0.0, False),
        ('l_gripper', 27, 0.0, True),
    ],
    '/dev/ttyACM1': [
        ('r_shoulder_pitch', 10, 90.0, False),
        ('r_shoulder_roll', 11, 90.0, False),
        ('r_arm_yaw', 12, 0.0, False),
        ('r_elbow_pitch', 13, 0.0, False),
        ('r_forearm_yaw', 14, 0.0, False),
        ('r_wrist_pitch', 15, 0.0, False),
        ('r_wrist_roll', 16, 0.0, False),
        ('r_gripper', 17, 0.0, True),
    ],
}


class USB2AXController(RobotHardwareABC):
    def __init__(self) -> None:
        self.dxl_controllers = [
            DxlController(port, config) for port, config in reachy_config.items()
        ]

    def get_joint_names(self) -> List[str]:
        return sum([list(c.names) for c in self.dxl_controllers], [])

    def get_joint_positions(self) -> List[float]:
        return sum([list(c.present_positions) for c in self.dxl_controllers], [])


class DxlController:
    def __init__(self, port, config) -> None:
        self.io = DxlIO(port=port, use_sync_read=True)
        self.names, self.ids, self.offsets, self.directs = zip(*config)

        self.synced = Event()

        t = Thread(target=self.sync_loop)
        t.daemon = True
        t.start()

        self.synced.wait()

    def sync_loop(self):
        while True:
            raw_pos = self.io.get_present_position(self.ids)
            self.present_positions = [
                from_dxl_pos(p, self.directs[i], self.offsets[i])
                for i, p in enumerate(raw_pos)
            ]
            self.synced.set()


def from_dxl_pos(dxl_pos, direct, offset) -> float:
    return np.deg2rad((dxl_pos if direct else -dxl_pos) - offset)


def to_dxl_pos(pos, direct, offset) -> float:
    return (np.rad2deg(pos) + offset) * (1 if direct else -1)
