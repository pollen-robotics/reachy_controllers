import rclpy
from rclpy.node import Node
from sensor_msgs import msg

from sensor_msgs.msg import JointState

from reachy_msgs.srv import SetCompliant


class ArmCopy(Node):
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
        super().__init__('arm_copy')

        self.compliant_client = self.create_client(SetCompliant, 'set_compliant')
        self.compliant_client.wait_for_service()

        self.clock = self.get_clock()

        self.joint_state_subscription = self.create_subscription(
            msg_type=JointState,
            topic='joint_states',
            callback=self.on_joint_states,
            qos_profile=1,
        )
        self.joint_goals_publisher = self.create_publisher(JointState, 'joint_goals', 1)
        self.joint_goals = JointState()
        self.joint_goals.name = self.left_arm

        request = SetCompliant.Request()
        request.name = self.left_arm
        request.compliant = [False] * len(self.left_arm)

        future = self.compliant_client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(self)

            if future.done():
                print(future.result())
                break

    def on_joint_states(self, joint_state: JointState):
        joint_names = []
        joint_goals = []

        for i, (name, pos) in enumerate(zip(joint_state.name, joint_state.position)):
            if name.startswith('r_'):
                joint_names.append(name.replace('r_', 'l_'))

                if name in [
                    'r_shoulder_roll',
                    'r_arm_yaw',
                    'r_forearm_yaw',
                    'r_wrist_roll',
                    'r_gripper',
                ]:
                    joint_goals.append(-pos)
                else:
                    joint_goals.append(pos)

        self.joint_goals.header.stamp = self.clock.now().to_msg()
        self.joint_goals.position = joint_goals

        self.joint_goals_publisher.publish(self.joint_goals)


def main():
    rclpy.init()

    arm_copy = ArmCopy()
    rclpy.spin(arm_copy)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
