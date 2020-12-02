from threading import Thread
from typing import Dict, List

import numpy as np

from pypot.dynamixel import DxlIO


from .robot_hardware_interface import RobotHardwareABC

reachy_config = {
    '/dev/ttyACM1': [
        ('l_shoulder_pitch', 20, 90.0, True),
        ('l_shoulder_roll', 21, -90.0, False),
        ('l_arm_yaw', 22, 0.0, False),
        ('l_elbow_pitch', 23, 0.0, False),
        ('l_forearm_yaw', 24, 0.0, False),
        ('l_wrist_pitch', 25, 0.0, False),
        ('l_wrist_roll', 26, 0.0, False),
        ('l_gripper', 27, 0.0, True),
    ],
    '/dev/ttyACM0': [
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
        self.motor2controller = {}
        for c in self.dxl_controllers:
            for motor in c.names:
                self.motor2controller[motor] = c

    def get_joint_names(self) -> List[str]:
        return sum([list(c.names) for c in self.dxl_controllers], [])

    def get_joint_positions(self) -> List[float]:
        return sum([list(c.present_positions) for c in self.dxl_controllers], [])

    def set_goal_positions(self, goal_positions: Dict[str, float]) -> None:
        for name, goal in goal_positions.items():
            self.motor2controller[name].set_goal_position(name, goal)

    def set_compliance(self, compliances: Dict[str, bool]) -> bool:
        success = True

        for name, compliance in compliances.items():
            if not self.motor2controller[name].set_compliance(name, compliance):
                success = False

        return success


class DxlController:
    def __init__(self, port, config) -> None:
        self.io = DxlIO(port=port, use_sync_read=True)
        self.names, self.ids, self.offsets, self.directs = zip(*config)

        self.motor_index = {name: i for i, name in enumerate(self.names)}

        self.present_positions = list(self.io.get_present_position(self.ids))
        self.goal_positions = list(self.io.get_goal_position(self.ids))
        self.stiff = list(self.io.is_torque_enabled(self.ids))


        t = Thread(target=self.sync_loop)
        t.daemon = True
        t.start()

    def set_compliance(self, name: str, compliance: bool) -> bool:
        i = self.motor_index[name]

        self.goal_positions[i] = self.present_positions[i]
        self.stiff[i] = not compliance

        return True

    def set_goal_position(self, name: str, goal_pos: float) -> None:
        self.goal_positions[self.motor_index[name]] = goal_pos

    def sync_loop(self) -> None:
        while True:
            raw_pos = self.io.get_present_position(self.ids)
            self.present_positions = [
                _from_dxl_pos(p, self.directs[i], self.offsets[i])
                for i, p in enumerate(raw_pos)
            ]

            goal_positions = {
                self.ids[i]: _to_dxl_pos(goal_pos, self.directs[i], self.offsets[i])
                for i, goal_pos in enumerate(self.goal_positions)
                if self.stiff[i]
            }
            if goal_positions:
                self.io.set_goal_position(goal_positions)

            compliants = [
                id
                for i, id in enumerate(self.ids)
                if not self.stiff[i]
            ]
            self.io.disable_torque(compliants)


def _from_dxl_pos(dxl_pos: float, direct: bool, offset: float) -> float:
    return np.deg2rad((dxl_pos if direct else -dxl_pos) - offset)


def _to_dxl_pos(pos: float, direct: bool, offset: float) -> float:
    return (np.rad2deg(pos) + offset) * (1 if direct else -1)
