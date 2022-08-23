"""
Gripper MX28 High-level Controller Node.

Listen to opening request and choose closing mode depending on the state: if a collision is detected or not.

This controller tries to automatically avoid gripper overheating by controlling torque.
This typically happens when holding an object, and adjusting the hold position.
It tunes the requested goal position and the PID depending on the chosen control mode.

Grippers can be in one of those states:

- holding an object: the goal position is calculated depending on the present position based on a theorical model
- empty: the operator is directly controlling the goal position, but the real goal position sent to the gripper is recalculated 
each dt based on the evolution of the requests

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
WIN_FILTER_LENGTH = 5

MAX_ACCEPTABLE_SERVO_SPEED = 100.0 * np.pi / 180

OPEN_POS = -0.87
CLOSE_POS = 0.35

MAX_TORQUE = 0.5

MAX_ERROR = 5.0 * np.pi / 180
SKIP_EARLY_DTS = 15
UPDATE_FREQ = 100  # Hz

MIN_DISTANCE_MOVEMENT = 0.1

P = 3.0
P_DIRECT_CONTROL = 5.0


def angle_diff(a, b):
    """Returns the smallest distance between 2 angles in radians."""
    d = a - b
    d = ((d + np.pi) % (2 * np.pi)) - np.pi
    return d


class GripperState:
    """Represent the current gripper state."""

    def __init__(self, name) -> None:
        # increment_per_dt: CONSTANT

        # direct (bool)

        # clock_wise (bool)
        # present_position + buff (10)
        # filtered_error (present - goal) # checker le sens

        # requested_goal_position: User command
        # previous_requested_goal_position: Last dt User command
        # interpolated_goal_position: Computed goal position for the current dt

        # Set by the update of the SM
        #
        # previously_in_collision
        # currently_in_collision
        # elapsed_dts_since_change_of_state
        # elapsed_dts_since_collision

        """Set up its base state."""
        self.name = name

        self.present_position = OPEN_POS
        self.requested_goal_position = OPEN_POS if name == 'r_gripper' else -OPEN_POS
        self.goal_position = self.requested_goal_position
        self._error = deque([0.0], maxlen=WIN_FILTER_LENGTH)

        self.current_case = 'resting'
        self._previous_case = 0
        self._early_dts_count = 0
        self._after_collision_dts_count = 0

        self._increment_per_dt = 0.0
        self._previous_requested_goal_position = self.requested_goal_position
        self._previous_goal_position = self.goal_position
        self._previous_present_position = self.present_position

        self._present_position_evolution = deque([0.0], maxlen=10)

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
        """Get goal position requested by the user at the previous dt."""
        return self._previous_requested_goal_position

    @previous_requested_goal_position.setter
    def previous_requested_goal_position(self, value):
        self._previous_requested_goal_position = value

    @property
    def previous_goal_position(self):
        """Get calculated goal position at the previous dt."""
        return self._previous_goal_position

    @previous_goal_position.setter
    def previous_goal_position(self, value):
        self._previous_goal_position = value

    @property
    def increment_per_dt(self):
        """Get expected distance travelled by the gripper in a dt."""
        return self._increment_per_dt

    @increment_per_dt.setter
    def increment_per_dt(self, value):
        self._increment_per_dt = value

    @property
    def filtred_error(self):
        """Filter the position error (used to detect forcing)."""
        return np.mean(self._error)

    def need_release(self):
        """Check whether the gripper should not be considered in collision anymore."""
        if self.name == 'r_gripper':
            if self.requested_goal_position < self.goal_position:
                self.in_collision = False
            if self.after_collision_dts_count > 10 and self.filtred_present_position_evolution < -0.002:
                self.in_collision = False
                self.after_collision_dts_count = 0
        else:
            if self.requested_goal_position > self.goal_position:
                self.in_collision = False
            if self.after_collision_dts_count > 10 and self.filtred_present_position_evolution > 0.002:
                self.in_collision = False
                self.after_collision_dts_count = 0


    @property
    def previous_case(self):
        """Get gripper state in previous action."""
        return self._previous_case

    @previous_case.setter
    def previous_case(self, value):
        self._previous_case = value

    @property
    def early_dts_count(self):
        """Count dts to ignore in the acceleration phase while moving."""
        return self._early_dts_count

    @early_dts_count.setter
    def early_dts_count(self, value):
        self._early_dts_count = value

    @property
    def after_collision_dts_count(self):
        """Count dts to ignore in release detection after collision has been detected."""
        return self._after_collision_dts_count

    @after_collision_dts_count.setter
    def after_collision_dts_count(self, value):
        self._after_collision_dts_count = value

    @property
    def present_position_evolution(self):
        """Get last present position of gripper."""
        return self._present_position_evolution[-1]

    @present_position_evolution.setter
    def present_position_evolution(self, value):
        self._present_position_evolution.append(value)
    
    @property
    def filtred_present_position_evolution(self):
        """Filter the present position evolution to detect movements of gripper."""
        return (self._present_position_evolution[0] - self._present_position_evolution[-1])


class GrippersController(Node):
    """Grippers High-level controller node."""

    def __init__(self):
        """Prepare the GrippersController node.

        Listen to topic /grippers for new opening/force commands.
        Create service client for compliancy and PID tuning.

        """
        super().__init__('grippers_controller')
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

        # TODO: introspection pour savoir quel gripper ?

        self.grippers = {
            'l_gripper': GripperState(name='l_gripper'),
            'r_gripper': GripperState(name='r_gripper'),
        }

        def gripper_state_update_thread():
            for gripper in self.grippers.values():
                self.set_pid(gripper, p=P_DIRECT_CONTROL, i=0.0, d=0.0)

            while rclpy.ok():
                self.gripper_state_update()
                time.sleep(1 / UPDATE_FREQ)

        # TODO: Use ROS timer?
        self.gripper_state_thread = Thread(target=gripper_state_update_thread)
        self.gripper_state_thread.daemon = True
        self.gripper_state_thread.start()

        self.logger.info('Node ready!')

    def grippers_callback(self, msg: Gripper):
        """Get latest Gripper msg from /grippers."""
        # TODO: deals with torque_limit
        for name, goal_position in zip(msg.name, msg.goal_position):
            if name not in self.grippers:
                continue
            gripper = self.grippers[name]
            gripper.previous_requested_goal_position = gripper.requested_goal_position
            gripper.requested_goal_position = goal_position

    def joint_states_callback(self, msg: JointState):
        """Get latest JointState from /joint_states."""
        for name, position in zip(msg.name, msg.position):
            if name not in self.grippers:
                continue

            gripper = self.grippers[name]
            gripper._previous_present_position = gripper.present_position
            gripper.present_position = position
            gripper.error = gripper.goal_position - gripper.present_position

            gripper.present_position_evolution = gripper.present_position

    def gripper_state_update(self):
        """Update grippers state machine."""
        for gripper in self.grippers.values():

            if(gripper.previously_in_collision and not gripper.in_collision):
                self.set_pid(gripper, p=P_DIRECT_CONTROL, i=0.0, d=0.0)
                gripper.previously_in_collision = False

            elif(gripper.in_collision and not gripper.previously_in_collision):
                self.set_pid(gripper, p=P, i=0.0, d=0.0)
                gripper.previously_in_collision = True

            if gripper.in_collision:
                self.close_fake_error(gripper)
                gripper.after_collision_dts_count += 1
                gripper.need_release()

            else:
                self.close_smart(gripper, gripper.requested_goal_position)
                self.check_error(gripper)

        self.publish_goals()

    def publish_goals(self):
        """Publish new /goal_states for grippers."""
        goals = JointState()

        for gripper in self.grippers.values():
            goals.name.append(gripper.name)

            if(gripper.in_collision):
                goals.position.append(gripper.goal_position)
            else:
                goals.position.append(gripper.requested_goal_position)

        if goals.name:
            self.logger.debug(f'Publish "{goals}" to "{self.joint_goals_publisher.topic_name}".')
            self.joint_goals_publisher.publish(goals)

    def set_pid(self, gripper, p, i, d):
        """Set gripper PID to new value."""
        gains = PidGains()
        gains.p = p
        gains.i = i
        gains.d = d

        pid = SetJointPidGains.Request()
        pid.name.append(gripper.name)
        pid.pid_gain.append(gains)

        resp = self.pid_gains_client.call(pid)
        self.logger.info(f'PID gains {(p, i, d)} set for "{gripper.name}" with resp "{resp.success}".')

    def close_fake_error(self, gripper, max_torque=MAX_TORQUE, dt=1/UPDATE_FREQ, p=P):
        """Depending on desired torque, calculate appropriate goal_position to hold the object."""

        current_pos = gripper.present_position

        # Calculating a goal pos so that if the error remains constant, then the output torque is max_torque.
        model_goal_pos = (np.pi/180)*(14*max_torque + 0.178)/p

        if(gripper.name == 'r_gripper'):
            goal_pos = model_goal_pos + current_pos
        else:
            goal_pos = - model_goal_pos + current_pos
        
        gripper.goal_position = goal_pos


    def close_smart(self, gripper, goal_pos, speed=MAX_ACCEPTABLE_SERVO_SPEED, dt=1/UPDATE_FREQ):
        """Evaluate request from teleop. Set possible goal_position depending on the requested target."""

        gripper.increment_per_dt = np.sign(angle_diff(goal_pos, gripper.present_position)) * speed * dt

        gripper.previous_case = gripper.current_case

        if(abs(goal_pos - gripper.previous_requested_goal_position) < MIN_DISTANCE_MOVEMENT):
            goal_pos = gripper.previous_requested_goal_position

        if(gripper.increment_per_dt < 0.0 and goal_pos <= gripper.previous_requested_goal_position):
    
            if(gripper.name == 'r_gripper'):
                gripper.current_case = 'opening'
            else:
                gripper.current_case = 'closing'

            current_goal_pos = gripper.goal_position
            current_goal_pos += gripper.increment_per_dt
            if(current_goal_pos < goal_pos):
                current_goal_pos = goal_pos
            
            gripper.goal_position = current_goal_pos

        elif(gripper.increment_per_dt > 0.0 and goal_pos >= gripper.previous_requested_goal_position):

            if(gripper.name == 'r_gripper'):
                gripper.current_case = 'closing'
            else:
                gripper.current_case = 'opening'

            current_goal_pos = gripper.goal_position
            current_goal_pos += gripper.increment_per_dt
            if(current_goal_pos > goal_pos):
                current_goal_pos = goal_pos

            gripper.goal_position = current_goal_pos

        if(gripper.current_case == gripper.previous_case):
            gripper.early_dts_count += 1
        else:
            gripper.early_dts_count = 0


    def check_error(self, gripper, max_error=MAX_ERROR, skip_early_dts=SKIP_EARLY_DTS):
        """Check if a collision occurred while trying to reach the goal position."""
        if(gripper.name == 'r_gripper'):
            if (gripper.filtred_error > max_error and gripper.requested_goal_position > gripper.present_position and gripper.early_dts_count > skip_early_dts):
                print("Collision detected on right gripper!")
                gripper.in_collision = True
                gripper.current_case = 'holding'
        else:
            if (gripper.filtred_error < -max_error and gripper.requested_goal_position < gripper.present_position and gripper.early_dts_count > skip_early_dts):
                print("Collision detected on left gripper!")
                gripper.in_collision = True
                gripper.current_case = 'holding'



def main(args=None):
    """Run gripper controller main loop."""
    rclpy.init(args=args)

    gripper = GrippersController()
    rclpy.spin(gripper)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
