"""Get the full state of the specified joints."""
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from reachy_msgs.msg import PidGains
from reachy_msgs.srv import GetJointFullState


class FullJointsStateClient(Node):
    """Node for /get_joint_full_state client."""

    def __init__(self) -> None:
        """Set up the node."""
        super().__init__('get_joint_full_state_client')

    def call_get_joint_full_state_async(self) -> Future:
        """Run the async service call."""
        client = self.create_client(srv_type=GetJointFullState, srv_name='get_joint_full_state')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        return client.call_async(GetJointFullState.Request())


def _repr_pid(pid: PidGains) -> str:
    if pid.p != 0:
        return f'{pid.p, pid.i, pid.d}'
    else:
        return f'{pid.cw_compliance_margin, pid.ccw_compliance_margin, pid.cw_compliance_slope, pid.ccw_compliance_slope}'


def main():
    """Run main loop."""
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--joints', type=str, nargs='+', default=None)
    args = parser.parse_args()

    rclpy.init()

    full_state_joint_client = FullJointsStateClient()
    future = full_state_joint_client.call_get_joint_full_state_async()

    rclpy.spin_until_future_complete(full_state_joint_client, future)
    result: GetJointFullState.Response = future.result()

    for (name, pos, temperature, compliant, speed_limit, torque_limit, pid) in zip(
        result.name, result.present_position, result.temperature, result.compliant,
        result.speed_limit, result.torque_limit, result.pid_gain,
    ):
        if args.joints is None or name in args.joints:
            print(f'Joint "{name}"')
            print(f'\tPosition={pos}')
            print(f'\tTemperature={temperature}')
            print(f'\tCompliant={compliant}')
            print(f'\tSpeed limit={speed_limit}')
            print(f'\tTorque limit={torque_limit}')
            print(f'\tPID={_repr_pid(pid)}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
