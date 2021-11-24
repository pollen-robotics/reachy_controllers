import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from reachy_msgs.msg import PidGains
from reachy_msgs.srv import GetJointFullState
from reachy_msgs.srv import SetFanState, SetJointCompliancy, SetJointPidGains


class MockupController(Node):
    def __init__(self, state_pub_rate=100):
        super().__init__('mockup_controller')

        self.logger = self.get_logger()
        self.clock = self.get_clock()

        self.joint_state = JointState()
        self.setup_joints()
        self.id4joint = {joint: i for i, joint in enumerate(self.joint_names)}

        self.joint_state_publisher = self.create_publisher(
            msg_type=JointState,
            topic='joint_states',
            qos_profile=5,
        )
        self.joint_state.name = self.joint_names
        self.joint_state_pub_timer = self.create_timer(
            timer_period_sec=1/state_pub_rate,
            callback=self.publish_joint_states,
        )
        self.logger.info(f'Setup "{self.joint_state_publisher.topic_name}" publisher ({state_pub_rate:.1f}Hz).')

        self.joint_goal_subscription = self.create_subscription(
            msg_type=JointState,
            topic='joint_goals',
            callback=self.on_joint_goals,
            qos_profile=5,
        )
        self.logger.info(f'Subscribe to "{self.joint_goal_subscription.topic_name}".')

        self.get_joint_full_state_srv = self.create_service(
            srv_type=GetJointFullState,
            srv_name='get_joint_full_state',
            callback=self.get_joint_full_state,
        )
        self.logger.info(f'Create "{self.get_joint_full_state_srv.srv_name}" service.')

        self.set_compliant_srv = self.create_service(
            srv_type=SetJointCompliancy,
            srv_name='set_joint_compliancy',
            callback=self.set_joint_compliancy,
        )
        self.logger.info(f'Create "{self.set_compliant_srv.srv_name}" service.')

        self.set_pid_srv = self.create_service(
            srv_type=SetJointPidGains,
            srv_name='set_joint_pid',
            callback=self.set_joint_pid,
        )
        self.logger.info(f'Create "{self.set_pid_srv.srv_name}" service.')

        self.fan_srv = self.create_service(
            srv_type=SetFanState,
            srv_name='set_fan_state',
            callback=self.set_fan_state,
        )
        self.logger.info(f'Create "{self.fan_srv.srv_name}" service.')

    def setup_joints(self):
        self.joint_state.position = [0.0 for _ in self.joint_names]


    @property
    def joint_names(self):
        return (
            'l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw', 'l_elbow_pitch', 
            'l_forearm_yaw', 'l_wrist_pitch', 'l_wrist_roll', 'l_gripper',
            'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw', 'r_elbow_pitch', 
            'r_forearm_yaw', 'r_wrist_pitch', 'r_wrist_roll', 'r_gripper',
            'l_antenna', 'r_antenna',
            'orbita_roll', 'orbita_pitch', 'orbita_yaw',
        )


    def publish_joint_states(self):
        self.joint_state.header.stamp = self.clock.now().to_msg()
        self.joint_state_publisher.publish(self.joint_state)

    def on_joint_goals(self, msg: JointState) -> None:
        for pos, name in zip(msg.position, msg.name):
            self.joint_state.position[self.id4joint[name]] = pos
        for vel, name in zip(msg.velocity, msg.name):
            self.joint_state.velocity[self.id4joint[name]] = pos
        for eff, name in zip(msg.effort, msg.name):
            self.joint_state.effort[self.id4joint[name]] = pos

    def get_joint_full_state(self,
                             request: GetJointFullState.Request,
                             response: GetJointFullState.Response,
                             ) -> GetJointFullState.Response:
        """Handle GetJointsFullState service request."""
        response.name = self.joint_names
        response.present_position = self.joint_state.position
        response.present_speed = self.joint_state.velocity
        response.present_load = self.joint_state.effort

        response.temperature = [37.5 for _ in self.joint_names]
        response.compliant = [False for _ in self.joint_names]
        response.goal_position = self.joint_state.position
        response.speed_limit = [0.0 for _ in self.joint_names]
        response.torque_limit = [100.0 for _ in self.joint_names]
        response.pid_gain = [PidGains(p=1.0, i=0.0, d=0.0) for _ in self.joint_names]

        return response

    def set_joint_compliancy(self,
                             request: SetJointCompliancy.Request,
                             response: SetJointCompliancy.Response,
                             ) -> SetJointCompliancy.Response:
        """Handle SetCompliant service request."""
        response.success = True
        return response

    def set_joint_pid(self,
                      request: SetJointPidGains.Request,
                      response: SetJointPidGains.Response,
                      ) -> SetJointPidGains:
        """Handle SetJointPidGains service request."""
        response.success = True
        return response

    def set_fan_state(self,
                      request: SetFanState.Request,
                      response: SetFanState.Response
                      ) -> SetFanState.Response:
        """Service fan callback."""
        response.success = True
        return response


def main() -> None:
    rclpy.init()

    mockup_controller = MockupController()
    rclpy.spin(mockup_controller)


if __name__ == '__main__':
    main()
