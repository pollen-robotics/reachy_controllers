"""
Example showing usage of /set_fan_state service and /fan_states topic.

The fans of each arm are successively turned on and off.
"""
import logging
import time

import rclpy
from rclpy.node import Node

from reachy_msgs.msg import FanState
from reachy_msgs.srv import SetFanState


class FanSwitcher(Node):
    """Node switching on and off the fans."""

    fans_names = []

    def __init__(self) -> None:
        """Initialize node, create client to /set_fan_state service and subscriber to /fan_states."""
        super().__init__('fan_switcher')

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger()

        self.fan_state_sub = self.create_subscription(
            msg_type=FanState,
            topic='fan_states',
            callback=self.fan_state_sub_cb,
            qos_profile=1,
            )
        self.fan_state_client = self.create_client(SetFanState, 'set_fan_state')
        self.fan_state_client.wait_for_service()

        self.logger.info('Waiting for the fans names from /fan_states...')
        rclpy.spin_once(self)

        while True:
            if self.fans_names != []:
                break

        for f_n in self.fans_names:
            if f_n == 'neck_fan':
                continue
            self._send_fan_request(f_n, True)
            self.logger.info(f'Turning on {f_n} for 5 seconds.')
            time.sleep(5.0)
            self._send_fan_request(f_n, False)
            self.logger.info(f'Turning off {f_n}.')

    def _send_fan_request(self, name: str, state: bool):
        request = SetFanState.Request(name=[name], state=[state])
        future = self.fan_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def fan_state_sub_cb(self, msg: FanState):
        """Recover the fans names. Callback for subscriber to /fan_states."""
        self.fans_names = msg.name


def main():
    """Run main loop."""
    rclpy.init()

    fan_switcher = FanSwitcher()
    rclpy.spin_once(fan_switcher)

    fan_switcher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
