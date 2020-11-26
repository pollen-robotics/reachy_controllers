"""
Joint State Controller Node.

- publish /joint_states at the specified rate (default: 100Hz)

"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from .robot_hardware_interface import RobotHardwareABC


class JointStateController(Node):
    """
    Joint State Controller Node.

        - publish /joint_states at the specified rate (default: 100Hz)
    """

    def __init__(self, robot_hardware: RobotHardwareABC, rate: float = 100.0) -> None:
        """Set up Node and create publisher."""
        super().__init__('joint_state_controller')

        self.robot_hardware = robot_hardware

        self.clock = self.get_clock()

        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 5)
        self.joint_state = JointState()
        self.joint_state.name = self.robot_hardware.get_joint_names()
        self.publish_timer = self.create_timer(timer_period_sec=1/rate, callback=self.publish_joint_states)

    def publish_joint_states(self) -> None:
        """Publish up-to-date JointState msg on /joint_states."""
        self.joint_state.header.stamp = self.clock.now().to_msg()
        self.joint_state.position = self.robot_hardware.get_joint_positions()

        self.joint_state_publisher.publish(self.joint_state)


def main() -> None:
    """Run joint state controller main loop."""
    from .robot_hardware_interface.usb2ax_controller import USB2AXController

    rclpy.init()

    joint_state_controller = JointStateController(
        robot_hardware=USB2AXController(),
    )
    rclpy.spin(joint_state_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
