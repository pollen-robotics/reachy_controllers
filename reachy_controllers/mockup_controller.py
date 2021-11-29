import time
from typing import List, Optional

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from reachy_msgs.msg import PidGains
from reachy_msgs.srv import GetJointFullState
from reachy_msgs.srv import SetFanState, SetJointCompliancy, SetJointPidGains


class MockupController(Node):
    def __init__(self, state_pub_rate=100):
        super().__init__('mockup_controller')

        self.logger = self.get_logger()
        self.clock = self.get_clock()

        self._joint_names: Optional[List[str]] = None

        cli = self.create_client(GetParameters, '/forward_position_controller/get_parameters')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.logger.warning(f'Waiting for {cli.srv_name} to get the joint names...')
        
        req = GetParameters.Request()
        req.names = ['joints']
        fut = cli.call_async(req)

        while not fut.done():
            rclpy.spin_once(self, timeout_sec=1)

        self._joint_names = fut.result().values[0].string_array_value
        self.logger.info(f'Got command joints {self._joint_names}')

        self.forward_command = Float64MultiArray()
        self.forward_command.data = [0.0 for _ in self.joint_names]
        self.id4joint = {joint: i for i, joint in enumerate(self.joint_names)}

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

        self.forward_command_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', qos_profile=5,
        )

    @property
    def joint_names(self):
        return self._joint_names

    def on_joint_goals(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.forward_command.data[self.id4joint[name]] = pos
        self.forward_command_pub.publish(self.forward_command)

    def get_joint_full_state(self,
                             request: GetJointFullState.Request,
                             response: GetJointFullState.Response,
                             ) -> GetJointFullState.Response:
        """Handle GetJointsFullState service request."""
        response.name = self.joint_names
        response.present_position = [0.0 for _ in self.joint_names]
        response.present_speed = [0.0 for _ in self.joint_names]
        response.present_load = [0.0 for _ in self.joint_names]

        response.temperature = [37.5 for _ in self.joint_names]
        response.compliant = [False for _ in self.joint_names]
        response.goal_position = [0.0 for _ in self.joint_names]
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
