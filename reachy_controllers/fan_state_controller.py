"""Fan handler service."""
from reachy_msgs.srv import ManageFan
from reachy_mockup_hardware.fan import MockupRobotFans

import rclpy
from rclpy import Node


class FanService(Node):
    """Service node for fan managing."""

    def __init__(self, robot_hardware):
        """Create a fan service."""
        super().__init__('fan_service')
        self.srv = self.create_service(
            srv_type=ManageFan,
            srv_name='fan_manager',
            callback=self.manage_fan_cb
        )
        self.robot_hardware = robot_hardware

    def manage_fan_cb(self,
                      request: ManageFan.Request,
                      response: ManageFan.Response
                      ) -> ManageFan.Response:
        """Service's callback."""
        fan_names = request.name
        fan_states = request.mod
        for fan, state in zip(fan_names, fan_states):
            if state:
                self.robot_hardware.fans[fan].on()
            else:
                self.robot_hardware.fans[fan].off()


def main(args=None):
    """Run service node."""
    rclpy.init(args=args)

    fan_service = FanService(robot_hardware=MockupRobotFans)
    rclpy.spin(fan_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
