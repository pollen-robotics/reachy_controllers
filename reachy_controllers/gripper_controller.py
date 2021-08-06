import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from reachy_msgs.msg import PidGains
from reachy_msgs.srv import SetJointCompliancy, SetJointPidGains
from reachy_msgs.msg import Gripper
import sys
import numpy as np
import time
from threading import Event

# Constants are defined for the right gripper
# Use the opposite for the left one
CLOSED_ANGLE = 0.3
OPEN_ANGLE = -1.0
MAX_ANGLE_FORCE = 8.0  # Angle offset to the goal position to "simulate" a force using the compliance slope
ANGLE_ERROR = 10


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        self.goal_publisher = self.create_publisher(JointState, 'joint_goals', 1)
        self.compliancy_client = self.create_client(SetJointCompliancy, 'set_joint_compliancy')
        while not self.compliancy_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service compliance not available, waiting again...')

        self.pid_gains_client = self.create_client(SetJointPidGains, 'set_joint_pid')
        while not self.pid_gains_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service pid not available, waiting again...')

        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_cb,
            10)

        self.cmd_sub = self.create_subscription(
            msg_type=Gripper,
            topic='grippers',
            callback=self.cmd_cb,
            qos_profile=5,
        )

        self.action_done_event = Event()
        self.gripper_pos = {}
        self.gripper_pos['l_gripper'] = None
        self.gripper_pos['r_gripper'] = None

        self.gripper_goal = {}
        self.gripper_goal['l_gripper'] = None
        self.gripper_goal['r_gripper'] = None

        self.gripper_vel = {}
        self.gripper_vel['l_gripper'] = None
        self.gripper_vel['r_gripper'] = None

        self.gripper_effort = {}
        self.gripper_effort['l_gripper'] = None
        self.gripper_effort['r_gripper'] = None

        self.state = {}
        self.state['l_gripper'] = 'open'
        self.state['r_gripper'] = 'open'

        self.closing_traj = {}
        self.closing_traj['l_gripper'] = []
        self.closing_traj['r_gripper'] = []

        self.closing_it = {}
        self.closing_it['l_gripper'] = []
        self.closing_it['r_gripper'] = []

        self.closing_force = {}
        self.closing_force['l_gripper'] = 0.0
        self.closing_force['r_gripper'] = 0.0

        self.client_futures = []

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        time.sleep(0.1)

        self.get_logger().info('gripper control node ready')

    def cmd_cb(self, msg):
        if msg.close_l_gripper:
            self.close_gripper('l_gripper', msg.l_gripper_force)
        else:
            self.open_gripper('l_gripper')

        if msg.close_r_gripper:
            self.close_gripper('r_gripper', msg.r_gripper_force)
        else:
            self.open_gripper('r_gripper')

    def joint_states_cb(self, msg):
        if 'l_gripper' in msg.name:
            lgi = msg.name.index('l_gripper')

            if len(msg.position) > 0:
                self.gripper_pos['l_gripper'] = msg.position[lgi]
            if len(msg.velocity) > 0:
                self.gripper_vel['l_gripper'] = msg.velocity[lgi]
            if len(msg.effort) > 0:
                self.gripper_effort['l_gripper'] = msg.effort[lgi]

        if 'r_gripper' in msg.name:
            rgi = msg.name.index('r_gripper')
            if len(msg.position) > 0:
                self.gripper_pos['r_gripper'] = msg.position[rgi]
            if len(msg.velocity) > 0:
                self.gripper_vel['r_gripper'] = msg.velocity[rgi]
            if len(msg.effort) > 0:
                self.gripper_effort['r_gripper'] = msg.effort[rgi]

    def timer_callback(self):
        self.handle_gripper('l_gripper')
        self.handle_gripper('r_gripper')

    def handle_gripper(self, g):
        """
        Kind of state machine for the grippers.

        In the closing state, a trajectory is followed and
        when a tracking error is to big, we decide that we have grasped something.
        """
        open_angle = OPEN_ANGLE if g == 'r_gripper' else -OPEN_ANGLE
        closed_angle = CLOSED_ANGLE if g == 'r_gripper' else -CLOSED_ANGLE

        if self.state[g] == 'opening':
            if self.gripper_pos[g] is None:
                self.set_pid(g, 1.0, 32.0)
                self.torque_on(g)
                self.goto(g, open_angle)
            else:
                error = np.abs(self.gripper_pos[g] - open_angle)
                if np.abs(np.degrees(error)) < 1.5:
                    self.torque_off(g)  # trying to save the motor...
                    self.state[g] = 'open'

        elif self.state[g] == "start_closing":
            if self.gripper_pos[g] is not None:
                self.closing_traj[g] = np.linspace(self.gripper_pos[g], closed_angle, 20)
                self.closing_it[g] = 0
                self.state[g] = 'closing'

            else:
                self.state[g] = 'open'

        elif self.state[g] == 'closing':
            self.get_logger().info('Closing: {}'.format(g))

            if self.closing_it[g] < len(self.closing_traj[g]) - 1:
                self.closing_it[g] += 1
                goal = self.closing_traj[g][self.closing_it[g]]
                self.goto(g, goal)

                error = np.abs(self.gripper_pos[g]-goal)
                self.get_logger().info('error: {} (pos: {} goal:{})'.format(
                    np.degrees(error), np.degrees(self.gripper_pos[g]), np.degrees(goal)),
                )
                if np.degrees(error) >= ANGLE_ERROR:
                    self.set_pid(g, 0.0, 254.0)
                    delta = np.radians(self.closing_force[g] * MAX_ANGLE_FORCE)
                    if g == 'l_gripper':
                        delta = -delta
                    self.goto(g, self.gripper_pos[g] + delta)
                    self.state[g] = 'closed'

            else:
                self.state[g] = 'closed'

        elif self.state[g] == 'closed':
            self.get_logger().info('Closed: {}'.format(g), once=True)

    def close_gripper(self, g, forcelevel=0.5):
        self.torque_on(g)
        self.set_pid(g, 0.0, 254.0)

        self.closing_force[g] = forcelevel
        self.state[g] = 'start_closing'
        self.get_logger().info('Closing gripper: {}'.format(g))

    def set_pid(self, g, margin, slope):
        pid = SetJointPidGains.Request()
        gains = PidGains()
        pid.name = [g]
        gains.cw_compliance_margin = margin
        gains.ccw_compliance_margin = margin
        gains.cw_compliance_slope = slope
        gains.ccw_compliance_slope = slope
        pid.pid_gain = [gains]

        self.client_futures.append(self.pid_gains_client.call_async(pid))

    def goto(self, g, pos):
        J = JointState()
        J.name = [g]
        J.position = [pos]
        self.goal_publisher.publish(J)

    def open_gripper(self, g):
        self.set_pid(g, 1.0, 32.0)
        self.torque_on(g)
        self.goto(g, OPEN_ANGLE if g == 'r_gripper' else -OPEN_ANGLE)
        self.state[g] = 'opening'

        self.get_logger().info('Opening gripper')

    def torque_off(self, g):
        self.get_logger().debug('Torque Off: {}'.format(g))

        req = SetJointCompliancy.Request()
        req.name.append(g)
        req.compliancy.append(True)
        self.client_futures.append(self.compliancy_client.call_async(req))

    def torque_on(self, g):
        self.get_logger().debug('Torque On: {}'.format(g))

        req = SetJointCompliancy.Request()
        req.name.append(g)
        req.compliancy.append(False)
        self.client_futures.append(self.compliancy_client.call_async(req))


def main(args=None):
    rclpy.init(args=args)

    gripper = GripperController()

    while rclpy.ok():

        try:
            rclpy.spin_once(gripper)
            incomplete_futures = []
            for f in gripper.client_futures:
                if f.done():
                    _ = f.result()
                else:
                    incomplete_futures.append(f)
            gripper.client_futures = incomplete_futures

        except KeyboardInterrupt:
            gripper.set_pid('l_gripper', 1.0, 32.0)
            gripper.set_pid('r_gripper', 1.0, 32.0)
            gripper.torque_off('l_gripper')
            gripper.torque_off('r_gripper')

            print('server stopped cleanly')
            gripper.destroy_node()
            rclpy.shutdown()
        except BaseException:
            gripper.set_pid('l_gripper', 1.0, 32.0)
            gripper.set_pid('r_gripper', 1.0, 32.0)
            gripper.torque_off('l_gripper')
            gripper.torque_off('r_gripper')

            print('exception in server:', file=sys.stderr)
            raise


if __name__ == '__main__':
    main()
