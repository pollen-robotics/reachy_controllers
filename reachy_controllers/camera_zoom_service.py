"""Service node to manage zoom for cameras."""
import rclpy
from rclpy.node import Node

from reachy_msgs.srv import ZoomCommand, SetZoomSpeed

from zoom_kurokesu import ZoomController


class ZoomControllerService(Node):
    """Main node creating the zoom services for cameras."""

    def __init__(self) -> None:
        """Set up the node and create the services."""
        super().__init__('zoom_controller_service')
        self.logger = self.get_logger()

        self.controller = ZoomController()
        self.command_service = self.create_service(
            ZoomCommand,
            'zoom_command',
            self.zoom_command_callback
            )
        self.logger.info(f'Launching "{self.command_service.srv_name}" service.')

        self.speed_service = self.create_service(
            SetZoomSpeed,
            'zoom_speed',
            self.zoom_speed_callback
            )
        self.logger.info(f'Launching "{self.speed_service.srv_name}" service.')

        self.logger.info('Node ready!')

    def zoom_command_callback(self, request: ZoomCommand.Request, response: ZoomCommand.Response) -> ZoomCommand.Response:
        """Handle zoom_command request."""
        req = request.zoom_command
        if req == 'homing':
            self.controller.homing(request.side)
            return response

        if req not in ('in', 'out', 'inter'):
            raise ValueError("Invalid command sent to zoom controller. Command must be in ('homing', 'in', 'out' or 'inter').")
        self.controller.send_zoom_command(request.side, request.zoom_command)
        return response

    def zoom_speed_callback(self, request: SetZoomSpeed.Request, response: SetZoomSpeed.Response) -> SetZoomSpeed.Response:
        """Handle zoom_speed request."""
        self.controller.set_speed(request.speed)
        return response


def main(args=None):
    """Run main loop."""
    rclpy.init(args=args)

    zoom_controller_service = ZoomControllerService()
    rclpy.spin(zoom_controller_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()