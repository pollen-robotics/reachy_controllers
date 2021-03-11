"""Service node to manage zoom for cameras."""

import rclpy
from rclpy.node import Node

from reachy_msgs.srv import SetCameraZoomLevel, SetCameraZoomSpeed

from zoom_kurokesu import ZoomController


class ZoomControllerService(Node):
    """Main node creating the zoom services for cameras."""

    def __init__(self) -> None:
        """Set up the node and create the services."""
        super().__init__('camera_zoom_controller_service')
        self.logger = self.get_logger()

        self.controller = ZoomController()
        self.command_service = self.create_service(
            SetCameraZoomLevel,
            'set_camera_zoom_level',
            self.zoom_command_callback,
        )
        self.logger.info(f'Launching "{self.command_service.srv_name}" service.')

        self.speed_service = self.create_service(
            SetCameraZoomSpeed,
            'set_camera_zoom_speed',
            self.zoom_speed_callback,
        )
        self.logger.info(f'Launching "{self.speed_service.srv_name}" service.')

        self.logger.info('Node ready!')

    def zoom_command_callback(self,
                              request: SetCameraZoomLevel.Request,
                              response: SetCameraZoomLevel.Response,
                              ) -> SetCameraZoomLevel.Response:
        """Handle set_camera_zoom_level request."""
        try:
            eye_side = {
                'left_eye': 'left',
                'right_eye': 'right',
            }[request.name]
        except AttributeError:
            self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")
            response.success = False
            return response

        if request.zoom_level == 'homing':
            self.controller.homing(eye_side)

        elif request.zoom_level in ('in', 'out', 'inter'):
            self.controller.send_zoom_command(eye_side, request.zoom_level)

        else:
            self.logger.warning("Invalid command sent to zoom controller (must be in ('homing', 'in', 'out' or 'inter')).")
            response.success = False
            return response

        response.success = True
        return response

    def zoom_speed_callback(self,
                            request: SetCameraZoomSpeed.Request,
                            response: SetCameraZoomSpeed.Response,
                            ) -> SetCameraZoomSpeed.Response:
        """Handle set_camera_zoom_speed request."""
        for name, speed in zip(request.name, request.speed):
            if name not in ['left_eye', 'right_eye']:
                self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")
                response.success = False
                return response

            self.controller.set_speed(speed)
        response.success = True
        return response


def main(args=None):
    """Run main loop."""
    rclpy.init(args=args)

    zoom_controller_service = ZoomControllerService()
    rclpy.spin(zoom_controller_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
