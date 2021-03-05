"""Examples showing how to turn all joints compliants."""

import rclpy
from rclpy.node import Node

from reachy_msgs.srv import SetJointCompliancy


class SetAllCompliant(Node):
    """Node responsible for turning all joints compliant."""

    motors = [
        'l_shoulder_pitch',
        'l_shoulder_roll',
        'l_arm_yaw',
        'l_elbow_pitch',
        'l_forearm_yaw',
        'l_wrist_pitch',
        'l_wrist_roll',
        'l_gripper',
        'r_shoulder_pitch',
        'r_shoulder_roll',
        'r_arm_yaw',
        'r_elbow_pitch',
        'r_forearm_yaw',
        'r_wrist_pitch',
        'r_wrist_roll',
        'r_gripper',
        'l_antenna',
        'r_antenna',
        'neck_disk_top',
        'neck_disk_middle',
        'neck_disk_bottom',
    ]

    def __init__(self) -> None:
        """Set up the node and connect to the set compliant service."""
        super().__init__('arm_copy')

        self.compliant_client = self.create_client(SetJointCompliancy, 'set_joint_compliancy')
        self.compliant_client.wait_for_service()

        request = SetJointCompliancy.Request()
        request.name = self.motors
        request.compliancy = [True] * len(self.motors)

        self.future = self.compliant_client.call_async(request)


def main():
    """Run main loop."""
    rclpy.init()

    all_compliant = SetAllCompliant()

    while rclpy.ok():
        rclpy.spin_once(all_compliant)

        if all_compliant.future.done():
            print(all_compliant.future.result())
            break

    rclpy.shutdown()


if __name__ == '__main__':
    main()
