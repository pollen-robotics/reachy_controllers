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
from enum import Enum
from threading import Thread
from matplotlib.pyplot import cla

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from reachy_msgs.msg import Gripper, PidGains
from reachy_msgs.srv import SetJointCompliancy, SetJointPidGains


# GRIPPER POS (for the right gripper)
OPEN_POS = -0.87
CLOSE_POS = 0.35

MAX_TORQUE = 0.5
P_SAFE_CLOSE = 3.0
P_DIRECT_CONTROL = 5.0

HISTORY_LENGTH = 10
UPDATE_FREQ = 100  # Hz
DT = 1 // UPDATE_FREQ
MOVING_SPEED = np.deg2rad(100)
INC_PER_DT = DT * MOVING_SPEED

SKIP_EARLY_DTS = 15
MIN_MOVING_DIST = 0.002
MAX_ERROR = 5.0 * np.pi / 180


class CollisionState(Enum):
    NO_COLLISION = 0
    ENTERING_COLLISION = 1
    STILL_COLLIDING = 2
    LEAVING_COLLISION = 3


class GripperState:
    """Represent the current gripper state."""
    def __init__(
        self,
        is_direct: bool,
        present_position: float, requested_goal_position: float,
    ) -> None:

        self.is_direct = is_direct

        self.present_position = deque([present_position], HISTORY_LENGTH)
        self.requested_goal_position = deque([requested_goal_position], HISTORY_LENGTH)

        self.interpolated_goal_position = deque([], HISTORY_LENGTH)
        self.error = deque([], HISTORY_LENGTH)
        self.in_collision = deque([], HISTORY_LENGTH)

        self.elapsed_dts_since_change_of_direction = 0
        self.elapsed_dts_since_collision = 0

    def update(self, new_present_position: float, new_requested_goal_position: float):
        self.present_position.append(new_present_position)

        if self.has_changed_direction(new_requested_goal_position):
            self.elapsed_dts_since_change_of_direction = 0

        self.requested_goal_position.append(new_requested_goal_position)

        # TODO: use enum instead
        collision_state = self.check_collision_state()

        if collision_state == CollisionState.NO_COLLISION:
            interpolated_goal_position = self.compute_fake_error_goal_position()

        elif collision_state == CollisionState.ENTERING_COLLISION:
            self.set_pid(gripper, p=P, i=0.0, d=0.0)
            interpolated_goal_position = self.compute_fake_error_goal_position()

        elif collision_state == CollisionState.STILL_COLLIDING:
            interpolated_goal_position = self.compute_close_smart_goal_position()

        elif collision_state == CollisionState.LEAVING_COLLISION:
            self.set_pid(gripper, p=P_DIRECT_CONTROL, i=0.0, d=0.0)
            interpolated_goal_position = self.compute_close_smart_goal_position()

        self.interpolated_goal_position.append(interpolated_goal_position)
        self.error.append(interpolated_goal_position - new_present_position)
        self.in_collision.append(collision_state in (CollisionState.ENTERING_COLLISION, CollisionState.STILL_COLLIDING))

        self.elapsed_dts_since_change_of_state += 1
        self.elapsed_dts_since_collision += 1

    def check_collision_state(self) -> CollisionState:
        if not hasattr(self, '_hidden_collision_state'):
            self._hidden_collision_state = CollisionState.NO_COLLISION

        if self.entering_collision():
            self._hidden_collision_state = CollisionState.STILL_COLLIDING
            return CollisionState.ENTERING_COLLISION
            
        if self.leaving_collision():
            self._hidden_collision_state = CollisionState.NO_COLLISION
            return CollisionState.LEAVING_COLLISION

        return self._hidden_collision_state

    def entering_collision(self) -> bool:
        if self.elapsed_dts_since_change_of_direction <= SKIP_EARLY_DTS:
            return False

        filtered_error = np.mean(self.error)
        return filtered_error > MAX_ERROR if self.is_direct else filtered_error < -MAX_ERROR

    def leaving_collision(self) -> bool:
        last_req_goal_pos = self.requested_goal_position[-1]
        last_interp_goal_pos = self.interpolated_goal_position[-1]

        # This condition is triggered by a direct user command to release our grasp
        user_request_to_release = (
            last_req_goal_pos < last_interp_goal_pos
            if self.is_direct else
            last_req_goal_pos > last_interp_goal_pos
        )

        # This condition is triggered because we are moving again, due to either:
        #   - because the object was removed
        #   - because it was a false detection in the first place
        moving_again = (
            self.elapsed_dts_since_collision > self.present_position.maxlen and
            abs(self.present_position[0] - self.present_position[-1]) > MIN_MOVING_DIST
        )

        return user_request_to_release or moving_again

    def has_changed_direction(self, new_goal_pos: float) -> bool:
        present_position = self.present_position[-1]
        last_goal_pos = self.requested_goal_position[-1]

        return np.sign(last_goal_pos - present_position) != np.sign(new_goal_pos - present_position)

    def compute_close_smart_goal_position(self) -> float:
        last_req_goal_pos = self.requested_goal_position[-1]
        goal_offset = INC_PER_DT * np.sign(last_req_goal_pos - self.present_position[-1])

        next_goal_pos = self.interpolated_goal_position[-1] + goal_offset

        return (
            min(next_goal_pos, last_req_goal_pos)
            if self.is_direct else
            max(next_goal_pos, last_req_goal_pos)
        )
        
    def compute_fake_error_goal_position(self) -> float:
        model_offset = np.deg2rad(14.0 * MAX_TORQUE + 0.178 / P_SAFE_CLOSE)
        if not self.is_direct:
            model_offset = -model_offset

        return model_offset + self.present_position[-1]


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
            'l_gripper': GripperState(is_direct=False),
            'r_gripper': GripperState(is_direct=True),
        }

        def gripper_state_update_thread():
            for gripper in self.grippers.values():
                self.set_pid(gripper, p=P_DIRECT_CONTROL, i=0.0, d=0.0)

            while rclpy.ok():
                self.gripper_state_update()
                time.sleep(DT)

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
        for gripper_state in self.grippers.values():
            gripper_state.update()

        self.publish_goals()
        self.publish_pids()

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


def main(args=None):
    """Run gripper controller main loop."""
    rclpy.init(args=args)

    gripper = GrippersController()
    rclpy.spin(gripper)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
