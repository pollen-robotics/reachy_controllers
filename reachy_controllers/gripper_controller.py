"""
Gripper High-level Controller Node.

Listen to opening/force messages for each gripper and automatically adjusts the gripper state in function.

This controller tries to automatically avoid gripper overheating by detecting forcing.
This typically happens when holding an object, and adjusting the hold position.
It also tunes the compliances and PIDs depending on the state.

Grippers can be in one of those states:

- resting: the compliance is turned on and the motor is resting while waiting for new commands
- moving: you have direct control of the gripper between its open and close position (using the opening scalar from 0 to 1)
- holding: a forcing has been detected (basically the gripper can not reach the desired goal position).
    A new hold position is computed from the current forcing position and the requested hold position.
    The PIDs are also adjusted to diminish overheating.
    This position will be held until the opening value is back to a release value.

"""

import time
from collections import deque
from threading import Thread

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from reachy_msgs.msg import Gripper, PidGains
from reachy_msgs.srv import SetJointCompliancy, SetJointPidGains

# Constants are defined for the right gripper
CLOSED_ANGLE = 0.4
OPEN_ANGLE = -0.85
MAX_ANGLE_FORCE = 0.15  # Angle offset to the goal position to "simulate" a force using the compliance slope
ANGLE_ERROR = 0.1

UPDATE_FREQ = 100  # Hz
WIN_FILTER_LENGTH = 20


class GripperState:
    """Represent the current gripper state."""

    def __init__(self, name) -> None:
        """Set up its base state."""
        self.name = name

        self.present_position = OPEN_ANGLE
        self._opening = deque([0.0], maxlen=WIN_FILTER_LENGTH)
        self._error = deque([0.0], maxlen=WIN_FILTER_LENGTH)

        self._closing_force = 0.0

        self.previous_state = 'resting'
        self.state = 'resting'

        self.open_angle = OPEN_ANGLE if name == 'r_gripper' else -OPEN_ANGLE
        self.closed_angle = CLOSED_ANGLE if name == 'r_gripper' else -CLOSED_ANGLE

    @property
    def opening(self):
        """Get the requested opening."""
        return self._opening[-1]

    @opening.setter
    def opening(self, value):
        self._opening.append(value)

    @property
    def filtered_opening(self):
        """Get the filtered opening (used to avoid high-frequency changed of states)."""
        return np.mean(self._opening)

    @property
    def closing_force(self):
        """Get the requested closing force."""
        return self._closing_force if self.name == 'r_gripper' else -self._closing_force

    @closing_force.setter
    def closing_force(self, value):
        self._closing_force = value

    @property
    def hold_position(self):
        """Compute the hold position based on the current position and the requested closing force."""
        return self.present_position + self.closing_force * MAX_ANGLE_FORCE

    @property
    def goal_position(self):
        """Compute the goal position based on the requested opening."""
        return self.open_angle + self.opening * (self.closed_angle - self.open_angle)

    @property
    def error(self):
        """Compute the position error of the gripper (diff between goal and present angle)."""
        return self._error[-1]

    @error.setter
    def error(self, value):
        self._error.append(value)

    @property
    def filtred_error(self):
        """Filter the position error (used to detect forcing)."""
        return np.mean(self._error)

    @property
    def error_derivative(self):
        """Compute the derivative of the position error (used to detect constant forcing)."""
        return np.abs(np.diff(self._error)).sum()

    def is_releasing(self):
        """Check whether the gripper is releasing its hold."""
        if self.name == 'r_gripper':
            return self.goal_position < 0.95 * self.hold_position
        else:
            return self.goal_position > 1.05 * self.hold_position


class GripperController(Node):
    """Gripper High-level controller node."""

    def __init__(self):
        """Prepare the GripperController node.

        Listen to topic /grippers for new opening/force commands.
        Create service client for compliancy and PID tuning.

        """
        super().__init__('gripper_controller')
        self.logger = self.get_logger()

        self.joint_states_sub = self.create_subscription(
            msg_type=JointState,
            topic='joint_states',
            callback=self.joint_states_callback,
            qos_profile=5,
        )
        self.logger.info(f'Subscribe to "{self.joint_states_sub.topic_name}".')

        self.joint_goals_publisher = self.create_publisher(
            msg_type=JointState,
            topic='joint_goals',
            qos_profile=5,
        )
        self.logger.info(f'Publish to "{self.joint_goals_publisher.topic_name}".')

        self.compliancy_client = self.create_client(SetJointCompliancy, 'set_joint_compliancy')
        while not self.compliancy_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('service compliance not available, waiting again...')
        self.logger.info(f'Client for "{self.compliancy_client.srv_name}" is available.')

        self.pid_gains_client = self.create_client(SetJointPidGains, 'set_joint_pid')
        while not self.pid_gains_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('service pid not available, waiting again...')
        self.logger.info(f'Client for "{self.pid_gains_client.srv_name}" is available.')

        self.grippers_sub = self.create_subscription(
            msg_type=Gripper,
            topic='grippers',
            callback=self.grippers_callback,
            qos_profile=5,
        )
        self.logger.info(f'Subscribe to "{self.grippers_sub.topic_name}".')

        self.grippers = {
            'l_gripper': GripperState(name='l_gripper'),
            'r_gripper': GripperState(name='r_gripper'),
        }

        def gripper_state_update_thread():
            for gripper in self.grippers.values():
                self.turn_off(gripper)

            while rclpy.ok():
                self.gripper_state_update()
                time.sleep(1 / UPDATE_FREQ)

        self.gripper_state_thread = Thread(target=gripper_state_update_thread)
        self.gripper_state_thread.daemon = True
        self.gripper_state_thread.start()

        self.logger.info('Node ready!')

    def grippers_callback(self, msg: Gripper):
        """Get latest Gripper msg from /grippers."""
        for name, opening, force in zip(msg.name, msg.opening, msg.force):
            if name not in self.grippers:
                continue
            gripper = self.grippers[name]
            gripper.opening = np.clip(opening, 0.0, 1.0)
            gripper.closing_force = force

    def joint_states_callback(self, msg: JointState):
        """Get latest JointState from /joint_states."""
        for name, position in zip(msg.name, msg.position):
            if name not in self.grippers:
                continue

            gripper = self.grippers[name]
            gripper.present_position = position
            gripper.error = np.abs(gripper.goal_position - gripper.present_position)

    def gripper_state_update(self):
        """Update grippers state machine."""
        for gripper in self.grippers.values():
            if gripper.previous_state == 'resting' and gripper.opening != 0.0:
                self.turn_on(gripper)
                gripper._opening.clear()
                gripper._opening.append(0.0)
                gripper.state = 'moving'

            elif gripper.previous_state == 'moving' and (0 < gripper.filtered_opening < 0.1):
                self.turn_off(gripper)
                gripper.state = 'resting'

            elif (
                gripper.previous_state == 'moving' and
                gripper.filtred_error > ANGLE_ERROR and
                gripper.error_derivative < ANGLE_ERROR
            ):
                self.logger.info('Forcing detected!')
                self.smart_hold(gripper)
                gripper.state = 'holding'

            elif gripper.previous_state == 'holding' and gripper.is_releasing():
                self.manual_control(gripper)
                gripper.state = 'moving'

            elif gripper.previous_state != gripper.state:
                raise EnvironmentError(f'Unknown transition {gripper.previous_state} -> {gripper.state}')

        self.publish_goals()

        for gripper in self.grippers.values():
            gripper.previous_state = gripper.state

    def turn_on(self, gripper):
        """Turn on the gripper."""
        self.logger.info(f'Turn on gripper "{gripper.name}"')
        self.set_pid(gripper, 1.0, 32.0)
        self.torque_mode(gripper, on=True)

    def turn_off(self, gripper):
        """Turn off gripper."""
        self.logger.info(f'Turn off gripper "{gripper.name}"')
        self.torque_mode(gripper, on=False)

    def smart_hold(self, gripper):
        """Trigger smart hold with fixed goal position matching the grasped object shape."""
        self.logger.info(f'Trigger smart holding for gripper "{gripper.name}"')
        self.set_pid(gripper, margin=0.0, slope=254.0)

    def manual_control(self, gripper):
        """Switch to manual control of the gripper."""
        self.logger.info(f'Back to manual control for gripper "{gripper.name}"')
        self.set_pid(gripper, margin=1.0, slope=32.0)

    def publish_goals(self):
        """Publish new /goal_states for grippers."""
        goals = JointState()

        for gripper in self.grippers.values():
            if gripper.state == 'moving':
                goals.name.append(gripper.name)
                goals.position.append(gripper.goal_position)
            elif gripper.state == 'holding' and gripper.previous_state == 'moving':
                self.logger.info(f'Maintain hold to {gripper.hold_position}')
                goals.name.append(gripper.name)
                goals.position.append(gripper.hold_position)

        if goals.name:
            self.logger.debug(f'Publish "{goals}" to "{self.joint_goals_publisher.topic_name}".')
            self.joint_goals_publisher.publish(goals)

    def set_pid(self, gripper, margin, slope):
        """Set gripper PID to new value (use margin/slope for AX-18)."""
        gains = PidGains()
        gains.cw_compliance_margin = margin
        gains.ccw_compliance_margin = margin
        gains.cw_compliance_slope = slope
        gains.ccw_compliance_slope = slope

        pid = SetJointPidGains.Request()
        pid.name.append(gripper.name)
        pid.pid_gain.append(gains)

        resp = self.pid_gains_client.call(pid)
        self.logger.info(f'PID gains {(margin, slope)} set for "{gripper.name}" with resp "{resp.success}".')

    def torque_mode(self, gripper, on):
        """Set gripper torque to new value."""
        req = SetJointCompliancy.Request()
        req.name.append(gripper.name)
        req.compliancy.append(not on)

        resp = self.compliancy_client.call(req)
        self.logger.info(f'Compliance mode "{not on}" set for "{gripper.name}" with resp "{resp.success}".')


def main(args=None):
    """Run gripper controller main loop."""
    rclpy.init(args=args)

    gripper = GripperController()
    rclpy.spin(gripper)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
