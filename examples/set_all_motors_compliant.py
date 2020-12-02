import rclpy
from rclpy.node import Node
from sensor_msgs import msg

from sensor_msgs.msg import JointState

from reachy_msgs.srv import SetCompliant


class SetAllCompliant(Node):
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
    ]

    def __init__(self) -> None:
        super().__init__('arm_copy')

        self.compliant_client = self.create_client(SetCompliant, 'set_compliant')
        self.compliant_client.wait_for_service()

        request = SetCompliant.Request()
        request.name = self.motors
        request.compliant = [True] * len(self.motors)

        self.future = self.compliant_client.call_async(request)


def main():
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
