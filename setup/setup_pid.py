"""Change the PID of the specified joint."""
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from reachy_msgs.msg import PidGains
from reachy_msgs.srv import SetJointPidGains


class SetPidClient(Node):
    """Node for /set_pid client."""

    def __init__(self) -> None:
        """Set up the node."""
        super().__init__('set_pid_client')

    def call_set_pid_async(self, joint_name: str, pids: List[float]) -> Future:
        """Run the async service call."""
        request = SetJointPidGains.Request()
        request.name = [joint_name]
        if len(pids) not in (3, 4):
            raise ValueError('PID gains should be either 3 or 4 values!')

        pid = PidGains()
        if len(pids) == 3:
            pid.p, pid.i, pid.d = pids
        else:
            pid.cw_compliance_margin, pid.ccw_compliance_margin, pid.cw_compliance_slope, pid.ccw_compliance_slope = pids
        request.pid_gain = [pid]

        client = self.create_client(srv_type=SetJointPidGains, srv_name='set_joint_pid')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        return client.call_async(request)


def main():
    """Run main loop."""
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('joint_name')
    parser.add_argument('pid', type=float, nargs='+')
    args = parser.parse_args()

    rclpy.init()

    set_pid_client = SetPidClient()
    future = set_pid_client.call_set_pid_async(args.joint_name, args.pid)

    rclpy.spin_until_future_complete(set_pid_client, future)
    print(future.result())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
