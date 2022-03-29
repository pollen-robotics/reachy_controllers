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
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from reachy_msgs.msg import JointTemperature
from reachy_msgs.msg import GripperMX28, PidGains
from reachy_msgs.srv import SetJointCompliancy, SetJointPidGains

# Constants are defined for the right gripper
WIN_FILTER_LENGTH = 5

MAX_SERVO_SPEED = 330.0 * np.pi / 180
MAX_ACCEPTABLE_SERVO_SPEED = 2000 * np.pi / 180

OPEN_POS = -0.87
CLOSE_POS = 0.35

MAX_TORQUE = 0.5

MAX_ERROR = 20.0 * np.pi / 180
SKIP_EARLY_DTS = 5
UPDATE_FREQ = 100  # Hz

MIN_DISTANCE_MOVEMENT = 0.1

P = 3.0
P_DIRECT_CONTROL = 5.0

servo_open_pos = -50.0 * np.pi / 180
servo_close_pos = -124.09 * np.pi / 180


def angle_diff(a, b):
    """Returns the smallest distance between 2 angles in radians."""
    d = a - b
    d = ((d + np.pi) % (2 * np.pi)) - np.pi
    return d

def sign(x):
    """Returns sign of value x."""
    if x >= 0:
        return 1
    return -1

class GripperMX28State:
    """Represent the current gripper state."""

    def __init__(self, name) -> None:
        """Set up its base state."""
        self.name = name

        self.present_position = OPEN_POS
        self.temperature = 45.0
        self.requested_goal_position = OPEN_POS if name == 'r_gripper' else -OPEN_POS
        self.goal_position = self.requested_goal_position
        self._error = deque([0.0], maxlen=WIN_FILTER_LENGTH)

        self._increment_per_dt = 0.0

        self.previously_in_collision = False
        self.in_collision = False

    @property
    def error(self):
        """Compute the position error of the gripper (diff between goal and present angle)."""
        return self._error[-1]

    @error.setter
    def error(self, value):
        self._error.append(value)

    @property
    def previous_requested_goal_position(self):
        """Compute the position error of the gripper (diff between goal and present angle)."""
        return self._previous_requested_goal_position

    @previous_requested_goal_position.setter
    def previous_requested_goal_position(self, value):
        self._previous_requested_goal_position = value

    @property
    def previous_goal_position(self):
        """Compute the position error of the gripper (diff between goal and present angle)."""
        return self._previous_goal_position

    @previous_goal_position.setter
    def previous_goal_position(self, value):
        self._previous_goal_position = value

    @property
    def increment_per_dt(self):
        """Compute the position error of the gripper (diff between goal and present angle)."""
        return self._increment_per_dt

    @increment_per_dt.setter
    def increment_per_dt(self, value):
        self._increment_per_dt = value
    
    @property
    def filtred_error(self):
        """Filter the position error (used to detect forcing)."""
        return np.mean(self._error)

    def need_release(self):
        """Check whether the gripper is releasing its hold."""
        if self.name == 'r_gripper':
            if self.requested_goal_position < self.goal_position:
                self.in_collision = False
        else:
            if self.requested_goal_position > self.goal_position:
                self.in_collision = False


class GripperMX28Controller(Node):
    """Gripper High-level controller node."""

    def __init__(self):
        """Prepare the GripperController node.

        Listen to topic /grippers for new opening/force commands.
        Create service client for compliancy and PID tuning.

        """
        super().__init__('gripperMX28_controller')
        self.logger = self.get_logger()

        self.joint_states_sub = self.create_subscription(
            msg_type=JointState,
            topic='joint_states',
            callback=self.joint_states_callback,
            qos_profile=5,
        )
        self.logger.info(f'Subscribe to "{self.joint_states_sub.topic_name}".')

        self.joint_temperatures_sub = self.create_subscription(
            msg_type=JointTemperature,
            topic='joint_temperatures',
            callback=self.joint_temperatures_callback,
            qos_profile=5,
        )
        self.logger.info(f'Subscribe to "{self.joint_temperatures_sub.topic_name}".')

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
            msg_type=GripperMX28,
            topic='grippersMX28',
            callback=self.grippers_callback,
            qos_profile=5,
        )
        self.logger.info(f'Subscribe to "{self.grippers_sub.topic_name}".')

        self.grippers = {
            # 'l_gripper': GripperMX28State(name='l_gripper'),
            'r_gripper': GripperMX28State(name='r_gripper'),
        }

        def gripper_state_update_thread():
            for gripper in self.grippers.values():
                self.turn_off(gripper)
                self.set_pid(gripper, p=P_DIRECT_CONTROL, i=0.0, d=0.0)

            while rclpy.ok():
                self.gripper_state_update()
                time.sleep(1 / UPDATE_FREQ)

        self.gripper_state_thread = Thread(target=gripper_state_update_thread)
        self.gripper_state_thread.daemon = True
        self.gripper_state_thread.start()

        self.logger.info('Node ready!')

    def grippers_callback(self, msg: GripperMX28):
        """Get latest Gripper msg from /grippersMX28."""
        print('callback GRASP')
        for name, goal_position in zip(msg.name, msg.goal_position):
            if name not in self.grippers:
                continue
            gripper = self.grippers[name]
            gripper.requested_goal_position = goal_position

    def joint_states_callback(self, msg: JointState):
        """Get latest JointState from /joint_states."""
        for name, position in zip(msg.name, msg.position):
            if name not in self.grippers:
                continue

            gripper = self.grippers[name]
            gripper._previous_present_position = gripper.present_position
            # if(position < 1.0):
            gripper.present_position = position
            gripper.error = np.abs(gripper.goal_position - gripper.present_position)
    
    def joint_temperatures_callback(self, msg: JointTemperature):
        """Get latest JointState from /joint_temperatures."""
        for name, temperature in zip(msg.name, msg.temperature):
            if name not in self.grippers:
                continue

            gripper = self.grippers[name]
            gripper.temperature = temperature

    def gripper_state_update(self):
        """Update grippers state machine."""
        for gripper in self.grippers.values():

            print(gripper.present_position)

    def turn_on(self, gripper):
        """Turn on the gripper."""
        self.logger.info(f'Turn on gripper "{gripper.name}"')
        self.set_pid(gripper, 1.0, 32.0)
        self.torque_mode(gripper, on=True)

    def turn_off(self, gripper):
        """Turn off gripper."""
        self.logger.info(f'Turn off gripper "{gripper.name}"')
        self.torque_mode(gripper, on=False)

    def publish_goals(self):
        """Publish new /goal_states for grippers."""
        goals = JointState()

        for gripper in self.grippers.values():
            # self.logger.info(f'Goal_position to {gripper.goal_position}')
            goals.name.append(gripper.name)
            goals.position.append(gripper.goal_position)

        if goals.name:
            self.logger.debug(f'Publish "{goals}" to "{self.joint_goals_publisher.topic_name}".')
            self.joint_goals_publisher.publish(goals)

    def set_pid(self, gripper, p, i, d):
        """Set gripper PID to new value (use margin/slope for AX-18)."""
        gains = PidGains()
        gains.p = float(p)
        gains.i = float(i)
        gains.d = float(d)

        pid = SetJointPidGains.Request()
        pid.name.append(gripper.name)
        pid.pid_gain.append(gains)

        resp = self.pid_gains_client.call(pid)
        # self.logger.info(f'PID gains {(p, i, d)} set for "{gripper.name}" with resp "{resp.success}".')

    def torque_mode(self, gripper, on):
        """Set gripper torque to new value."""
        req = SetJointCompliancy.Request()
        req.name.append(gripper.name)
        req.compliancy.append(not on)

        resp = self.compliancy_client.call(req)
        self.logger.info(f'Compliance mode "{not on}" set for "{gripper.name}" with resp "{resp.success}".')

    def close_fake_error(self, gripper, max_torque=0.5, dt=0.01, p=3.0, close_time=3):
        print('close_fake_error')
        # print("Setting p value at {}".format(p))
        # self.set_pid(gripper, p=p, i=0.0, d=0.0)
        t = 0
        print('pid set')
        while True:
            if t > close_time:
                break
            t += dt
            current_pos = gripper.present_position
            # Calculating a goal pos so that if the error remains constant, then the output torque is max_torque.
            goal_pos = -(14*max_torque + 0.178)/p + current_pos
            error = angle_diff(goal_pos, current_pos)
            # print("**Goal pos = {}. Error = {}. Current pos = {}".format(goal_pos, error, current_pos))
            gripper.goal_position = goal_pos
            self.publish_goals()
            time.sleep(dt)
        print('finish')
        return 0

    def close_smart(self, gripper, goal_pos, max_torque=0.5, speed=MAX_ACCEPTABLE_SERVO_SPEED, dt=0.01, max_error=3.0, skip_early_dts = 5):
        print('beginning of close_smart')
        p = 4.0
        normal_error_delay = 2.9
        normal_error_delay = speed*normal_error_delay/150
        # print("Setting p value at {}".format(p))
        #self.set_pid(gripper, p=p, i=0.0, d=0.0)
        initial_pos = gripper.present_position
        ang_dist = angle_diff(goal_pos, initial_pos)
        total_close_time = abs(ang_dist/speed)
        increment_per_dt = ang_dist/(total_close_time/dt)
        current_goal_pos = initial_pos
        collision = False

        print('increment_per_dt', increment_per_dt)
        # print("total_close_time {}".format(total_close_time))
        # print("ang_dist {}".format(ang_dist))
        print('close_smart')

        for i in range(math.floor(total_close_time/dt)):
            current_goal_pos += increment_per_dt
            gripper.goal_position = current_goal_pos
            current_pos = gripper.present_position
            error = angle_diff(current_goal_pos, current_pos)
            smart_error = angle_diff(current_goal_pos, current_pos-normal_error_delay) 
            # print("Goal pos = {}. Error = {}. Corrected error = {}. Current pos = {}".format(current_goal_pos, error, smart_error, current_pos))
            if i >= skip_early_dts:
                print('current_goal_pos', current_goal_pos)
                print('current_pos', current_pos)
                print('smart_error', smart_error)
                if abs(smart_error) > max_error:
                    print("Collision detected, activating model based torque control")
                    collision = True
                    self.close_fake_error(gripper, max_torque=max_torque, dt=0.01, p=3, close_time=5)
                    break
            self.publish_goals()
            time.sleep(dt)

        if not(collision):
            print('gonna return 0')
            gripper.goal_position = current_goal_pos
            return 0
        else:
            print('gonna return 1')
            return 1


def main(args=None):
    """Run gripper controller main loop."""
    rclpy.init(args=args)

    gripper = GripperMX28Controller()
    rclpy.spin(gripper)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
