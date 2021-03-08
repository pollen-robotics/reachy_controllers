"""Examples showing how to turn all joints compliants."""
from typing import List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from reachy_msgs.srv import SetJointCompliancy


class SetAllCompliant(Node):
    """Node responsible for turning all joints compliant."""

    def __init__(self) -> None:
        """Set up the node and connect to the set compliant service."""
        super().__init__('arm_copy')

        self.compliant_client = self.create_client(SetJointCompliancy, 'set_joint_compliancy')
        self.compliant_client.wait_for_service()

        self.joints_name = self.get_all_joints_name()

        request = SetJointCompliancy.Request()
        request.name = self.joints_name
        request.compliancy = [True] * len(self.joints_name)

        self.future = self.compliant_client.call_async(request)

    def get_all_joints_name(self) -> List[str]:
        """Retrieve all joints name."""
        def cb(msg: JointState):
            self.joints_name = msg.name

        joint_state_sub = self.create_subscription(
            msg_type=JointState, topic='/joint_states',
            callback=cb, qos_profile=5,
        )
        rclpy.spin_once(self)
        self.destroy_subscription(joint_state_sub)
        return self.joints_name


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
