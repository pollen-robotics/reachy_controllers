"""
Examples showing usage of /joint_states and /joint_goals topic.

The position of the right arm is read and set to the left arm.
"""


import rclpy
from rclpy.node import Node

from reachy_msgs.srv import SetJointCompliancy

from sensor_msgs.msg import JointState


class ArmCopy(Node):
    """Node reponsibles for copying movements applied to the left arm on the right one."""

    left_arm = [
        'l_shoulder_pitch',
        'l_shoulder_roll',
        'l_arm_yaw',
        'l_elbow_pitch',
        'l_forearm_yaw',
        'l_wrist_pitch',
        'l_wrist_roll',
        'l_gripper',
    ]
    right_arm = [
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
        """Set up the Node and create necessary pub/sub/services."""
        super().__init__('arm_copy')

        self.compliant_client = self.create_client(SetJointCompliancy, 'set_joint_compliancy')
        self.compliant_client.wait_for_service()

        self.clock = self.get_clock()

        self.joint_state_subscription = self.create_subscription(
            msg_type=JointState,
            topic='joint_states',
            callback=self.on_joint_states,
            qos_profile=1,
        )
        self.joint_goals_publisher = self.create_publisher(JointState, 'joint_goals', 5)
        self.joint_goals = JointState()

        request = SetJointCompliancy.Request()
        request.name = self.left_arm
        request.compliancy = [False] * len(self.left_arm)

        future = self.compliant_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def on_joint_states(self, joint_state: JointState):
        """Handle message callback for the /joint_states topic."""
        joint_names = []
        joint_goals = []

        for i, (name, pos) in enumerate(zip(joint_state.name, joint_state.position)):
            if name.startswith('r_'):
                name = f'l_{name[2:]}'
                joint_names.append(name)

                if name in [
                    'l_shoulder_roll',
                    'l_arm_yaw',
                    'l_forearm_yaw',
                    'l_wrist_roll',
                    'l_gripper',
                ]:
                    joint_goals.append(-pos)
                else:
                    joint_goals.append(pos)

        self.joint_goals.header.stamp = self.clock.now().to_msg()
        self.joint_goals.name = joint_names
        self.joint_goals.position = joint_goals
        self.joint_goals.velocity = [0.0 for _ in joint_names]

        self.joint_goals_publisher.publish(self.joint_goals)


def main():
    """Run main loop."""
    rclpy.init()

    arm_copy = ArmCopy()
    rclpy.spin(arm_copy)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
