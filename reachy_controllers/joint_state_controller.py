"""
Joint State Controller Node.

Exposes
 - all joints related information (pos/speed/load/temp)
 - joint compliancy service
 - force sensors
 - fan management

The access to the hardware is done through an HAL.

"""
import logging
from typing import Type

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Temperature

from reachy_pyluos_hal.joint_hal import JointLuos

from reachy_msgs.msg import ForceSensor
from reachy_msgs.msg import JointTemperature
from reachy_msgs.srv import GetJointFullState, SetJointCompliancy
from reachy_msgs.srv import SetFanState


class JointStateController(Node):
    """Joint State Controller Node."""

    def __init__(self, robot_hardware: Type[JointLuos],
                 state_pub_rate: float = 100.0,
                 temp_pub_rate: float = 0.1,
                 force_pub_rate: float = 10.0
                 ) -> None:
        """Set up the Node and the pub/sub/srv.

        Topic:
            - publish /joint_states at the specified rate (default: 100Hz)
            - subscribe to /joint_goals and forward the pos/vel/eff to the associated hal

            - publish /joint_temperatures at the specified rate (default: 0.1Hz)

            - publish /force_sensors at the specified rate (default: 10Hz)

        Service:
            - /get_joints_full_state GetJointFullState
            - /set_joint_compliancy SetJointCompliancy
            - /set_joint_pid SetJointPID (TODO: impl :))
            - /set_fan_state SetFanState
        """
        super().__init__('joint_state_controller')

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger()

        self.robot_hardware = robot_hardware(self.logger)
        self.robot_hardware.__enter__()

        self.force_sensor_names = self.robot_hardware.get_all_force_sensor_names()
        self.joint_names = self.robot_hardware.get_all_joint_names()

        self.clock = self.get_clock()

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
        self.logger.info(f'Setup "{self.joint_state_publisher.topic_name}" publisher ({state_pub_rate:.1f}Hz).')

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
        self.logger.info(f'Setup "{self.joint_temperature_publisher.topic_name}" publisher ({temp_pub_rate:.1f}Hz).')

        self.force_sensors_publisher = self.create_publisher(
            msg_type=ForceSensor,
            topic='force_sensors',
            qos_profile=5,
        )
        self.force_sensors = ForceSensor()
        self.force_sensors.name = self.force_sensor_names
        self.force_sensors_pub_timer = self.create_timer(
            timer_period_sec=1/force_pub_rate,
            callback=self.publish_force_sensors,
        )
        self.logger.info(f'Setup "{self.force_sensors_publisher.topic_name}" publisher ({force_pub_rate:.1f}Hz).')

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

        self.fan_srv = self.create_service(
            srv_type=SetFanState,
            srv_name='set_fan_state',
            callback=self.set_fan_state,
        )
        self.logger.info(f'Create "{self.fan_srv.srv_name}" service.')

        self.logger.info('Node ready!')

    def shutdown(self) -> None:
        """Clean and close the connection with the robot."""
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

    def publish_force_sensors(self) -> None:
        """Publish force gripper sensor values for both arm on /force_gripper."""
        self.force_sensors.header.stamp = self.clock.now().to_msg()
        self.force_sensors.force = self.robot_hardware.get_force(self.force_sensor_names)
        self.force_sensors_publisher.publish(self.force_sensors)

    def on_joint_goals(self, msg: JointState) -> None:
        """Handle new JointState goal by calling the robot hardware abstraction."""
        if msg.velocity:
            self.robot_hardware.set_goal_velocities(dict(zip(msg.name, msg.velocity)))
        if msg.effort:
            self.robot_hardware.set_goal_efforts(dict(zip(msg.name, msg.effort)))
        if msg.position:
            self.robot_hardware.set_goal_positions(dict(zip(msg.name, msg.position)))

    def get_joint_full_state(self,
                             request: GetJointFullState.Request,
                             response: GetJointFullState.Response,
                             ) -> GetJointFullState.Response:
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

    def set_joint_compliancy(self,
                             request: SetJointCompliancy.Request,
                             response: SetJointCompliancy.Response,
                             ) -> SetJointCompliancy.Response:
        """Handle SetCompliant service request."""
        compliances = dict(zip(request.name, request.compliancy))
        success = self.robot_hardware.set_compliance(compliances)

        response.success = success
        return response

    def set_fan_state(self,
                      request: SetFanState.Request,
                      response: SetFanState.Response
                      ) -> SetFanState.Response:
        """Service fan callback."""
        success = self.robot_hardware.set_fan_state(dict(zip(request.name, request.state)))
        response.success = success

        return response


def main() -> None:
    """Run joint state controller main loop."""
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
