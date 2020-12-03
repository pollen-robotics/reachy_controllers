"""
Joint State Controller Node.

- publish /joint_states at the specified rate (default: 100Hz)

"""

import rclpy
from rclpy.node import Node

from reachy_msgs.srv import SetCompliant

from reachy_ros_hal.joint import JointDynamixelABC

from sensor_msgs.msg import JointState


class JointStateController(Node):
    """
    Joint State Controller Node.

        - publish /joint_states at the specified rate (default: 100Hz)
    """

    def __init__(self, robot_hardware: JointDynamixelABC, rate: float = 100.0) -> None:
        """Set up Node and create publisher."""
        super().__init__('joint_state_controller')

        self.robot_hardware = robot_hardware

        self.clock = self.get_clock()

        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 1)
        self.joint_state = JointState()
        self.joint_state.name = self.robot_hardware.get_joint_names()
        self.publish_timer = self.create_timer(timer_period_sec=1/rate, callback=self.publish_joint_states)

        self.joint_goal_subscription = self.create_subscription(
            msg_type=JointState,
            topic='joint_goals',
            callback=self.on_joint_goals,
            qos_profile=1,
        )

        self.set_compliant_srv = self.create_service(
            srv_type=SetCompliant,
            srv_name='set_compliant',
            callback=self.set_compliant,
        )

    def publish_joint_states(self) -> None:
        """Publish up-to-date JointState msg on /joint_states."""
        self.joint_state.header.stamp = self.clock.now().to_msg()
        self.joint_state.position = self.robot_hardware.get_joint_positions()

        self.joint_state_publisher.publish(self.joint_state)

    def on_joint_goals(self, msg: JointState) -> None:
        goal_positions = dict(zip(msg.name, msg.position))
        self.robot_hardware.set_goal_positions(goal_positions)

    def set_compliant(self, request, response) -> bool:
        compliances = dict(zip(request.name, request.compliant))
        success = self.robot_hardware.set_compliance(compliances)

        response.success = success
        return response


def main() -> None:
    """Run joint state controller main loop."""
    from reachy_mockup_hardware.joint import MockupJointDynamixel

    rclpy.init()

    joint_state_controller = JointStateController(
        robot_hardware=MockupJointDynamixel(),
    )
    rclpy.spin(joint_state_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
