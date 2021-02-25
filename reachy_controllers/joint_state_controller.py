"""
Joint State Controller Node.

Exposes all joints related information (pos/speed/load/temp).
The access to the hardware is done through an HAL.

"""
from logging import Logger
import logging
from typing import Type

import rclpy
from rclpy.node import Node

from reachy_msgs.msg import JointTemperature, LoadSensor
from reachy_msgs.srv import GetJointsFullState, SetCompliant

from reachy_ros_hal.joint import JointABC

from sensor_msgs.msg import JointState, Temperature


class JointStateController(Node):
    """Joint State Controller Node."""

    def __init__(self, robot_hardware: Type[JointABC],
                 state_pub_rate: float = 100.0,
                 temp_pub_rate: float = 0.1,
                 fg_pub_rate: float = 10.0
                 ) -> None:
        """Set up the Node and the pub/sub/srv.

        Topic:
            - publish /joint_states at the specified rate (default: 100Hz)
            - publish /joint_temperatures at the specified rate (default: 0.1Hz)
            - publish /force_gripper at the specified rate (default: 10Hz)
            - subscribe to /joint_goals and forward the pos/vel/eff to the associated hal

        Service:
            - /get_joints_full_state GetJointsFullState
            - /set_compliant SetCompliant
            - /set_pid SetPID (TODO: impl :))
        """
        super().__init__('joint_state_controller')

        import logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger()

        self.robot_hardware = robot_hardware(self.logger)
        self.robot_hardware.__enter__()
        self.joint_names = self.robot_hardware.get_all_joint_names()

        self.clock = self.get_clock()

        self.logger.info(f'Setup "/joint_states" publisher ({state_pub_rate:.1f}Hz).')
        self.joint_state_publisher = self.create_publisher(
            msg_type=JointState,
            topic='joint_states',
            qos_profile=5,
        )
        self.joint_state = JointState()
        self.joint_state.name = self.joint_names
        self.joint_state_pub_timer = self.create_timer(
            timer_period_sec=1/state_pub_rate,
            callback=self.publish_joint_states,
        )

        self.logger.info(f'Setup "/joint_temperatures" publisher ({temp_pub_rate:.1f}Hz).')
        self.joint_temperature_publisher = self.create_publisher(
            msg_type=JointTemperature,
            topic='joint_temperatures',
            qos_profile=5,
        )
        self.joint_temperature = JointTemperature()
        self.joint_temperature.name = self.joint_names
        self.joint_temperature.temperature = [Temperature() for _ in self.joint_names]
        self.joint_temp_pub_timer = self.create_timer(
            timer_period_sec=1/temp_pub_rate,
            callback=self.publish_joint_temperatures,
        )

        self.logger.info(f'Setup "/force_gripper" publisher ({fg_pub_rate:.1f}Hz).')
        self.force_gripper_publisher = self.create_publisher(
            msg_type=LoadSensor,
            topic='force_gripper',
            qos_profile=5,
        )
        self.force_gripper = LoadSensor()
        self.force_gripper.side = ['right', 'left']
        self.force_gripper_pub_timer = self.create_timer(
            timer_period_sec=1/fg_pub_rate,
            callback=self.publish_force_gripper
        )

        self.logger.info('Subscribe to "/joint_goals".')
        self.joint_goal_subscription = self.create_subscription(
            msg_type=JointState,
            topic='joint_goals',
            callback=self.on_joint_goals,
            qos_profile=5,
        )

        self.logger.info('Create "/get_joint_full_state" service.')
        self.get_joint_full_state_srv = self.create_service(
            srv_type=GetJointsFullState,
            srv_name='get_joint_full_state',
            callback=self.get_joint_full_state,
        )

        self.logger.info('Create "/set_compliant" service.')
        self.set_compliant_srv = self.create_service(
            srv_type=SetCompliant,
            srv_name='set_compliant',
            callback=self.set_compliant,
        )

        self.logger.info('Node ready!')

    def shutdown(self) -> None:
        self.logger.info('Shutting down... Please wait!')
        self.robot_hardware.stop()

    def publish_joint_states(self) -> None:
        """Publish up-to-date JointState msg on /joint_states."""
        self.joint_state.header.stamp = self.clock.now().to_msg()

        positions = self.robot_hardware.get_joint_positions(self.joint_names)
        if positions is not None:
            self.joint_state.position = positions

        velocities = self.robot_hardware.get_joint_velocities(self.joint_names)
        if velocities is not None:
            self.joint_state.velocity = velocities

        efforts = self.robot_hardware.get_joint_efforts(self.joint_names)
        if efforts is not None:
            self.joint_state.effort = efforts

        self.joint_state_publisher.publish(self.joint_state)

    def publish_joint_temperatures(self) -> None:
        """Publish up-to-date JointTemperature msg on /jont_temperatures."""
        self.joint_temperature.header.stamp = self.clock.now().to_msg()

        for i, temp in enumerate(self.robot_hardware.get_joint_temperatures(self.joint_names)):
            self.joint_temperature.temperature[i].temperature = float(temp)

        self.joint_temperature_publisher.publish(self.joint_temperature)

    def publish_force_gripper(self) -> None:
        """Publish force gripper sensor values for both arm on /force_gripper."""
        self.force_gripper.header.stamp = self.clock.now().to_msg()
        self.force_gripper.load_value = self.robot_hardware.get_grip_force(self.force_gripper.side)
        self.force_gripper_publisher.publish(self.force_gripper)

    def on_joint_goals(self, msg: JointState) -> None:
        """Handle new JointState goal by calling the robot hardware abstraction."""
        if msg.position:
            self.robot_hardware.set_goal_positions(dict(zip(msg.name, msg.position)))
        if msg.velocity:
            self.robot_hardware.set_goal_velocities(dict(zip(msg.name, msg.velocity)))
        if msg.effort:
            self.robot_hardware.set_goal_efforts(dict(zip(msg.name, msg.effort)))

    def get_joint_full_state(self,
                             request: GetJointsFullState.Request,
                             response: GetJointsFullState.Response,
                             ) -> GetJointsFullState.Response:
        """Handle GetJointsFullState service request."""
        response.name = self.joint_names

        positions = self.robot_hardware.get_joint_positions(self.joint_names)
        if positions:
            response.present_position = positions

        velocities = self.robot_hardware.get_joint_velocities(self.joint_names)
        if velocities:
            response.present_speed = velocities

        efforts = self.robot_hardware.get_joint_efforts(self.joint_names)
        if efforts:
            response.present_load = efforts

        response.temperature = [float(temp) for temp in self.robot_hardware.get_joint_temperatures(self.joint_names)]
        response.compliant = self.robot_hardware.get_compliant(self.joint_names)
        response.goal_position = self.robot_hardware.get_goal_positions(self.joint_names)
        response.speed_limit = self.robot_hardware.get_goal_velocities(self.joint_names)
        response.torque_limit = self.robot_hardware.get_goal_efforts(self.joint_names)

        return response

    def set_compliant(self, request: SetCompliant.Request, response: SetCompliant.Response) -> SetCompliant.Response:
        """Handle SetCompliant service request."""
        compliances = dict(zip(request.name, request.compliant))
        success = self.robot_hardware.set_compliance(compliances)

        response.success = success
        return response


def main() -> None:
    """Run joint state controller main loop."""
    from reachy_pyluos_hal.joint_hal import JointLuos

    rclpy.init()

    joint_state_controller = JointStateController(
        robot_hardware=JointLuos,
    )

    try:
        rclpy.spin(joint_state_controller)
    except KeyboardInterrupt:
        joint_state_controller.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
