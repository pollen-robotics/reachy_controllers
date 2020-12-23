"""Fan handler service."""

from reachy_msgs.srv import ManageFan

import rclpy
from rclpy import Node


class FanService(Node):
    """Service node for fan state managing."""

    def __init__(self):
        """Create a fan service."""
        super().__init__('fan_service')
        self.srv = self.create_service(ManageFan, 'manage_fan', self.manage_fan_cb)

    def manage_fan_cb(self, request, response):
        """Service's callback."""
        fan_names = request.name
        fan_states = request.mod
        for n, s in zip(fan_names, fan_states):
            n.on() if s else n.off()


def main(args=None):
    """Run service node."""
    rclpy.init(args=args)

    fan_service = FanService()
    rclpy.spin(fan_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
