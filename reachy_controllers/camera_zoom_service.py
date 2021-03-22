"""Service node to manage zoom for cameras."""
import time

import rclpy
from rclpy.node import Node

from reachy_msgs.srv import GetCameraZoomLevel, SetCameraZoomLevel
from reachy_msgs.srv import GetCameraZoomSpeed, SetCameraZoomSpeed

from zoom_kurokesu import ZoomController


class ZoomControllerService(Node):
    """Main node creating the zoom services for cameras."""

    def __init__(self, default_zoom_speed: int = 30000, default_zoom_level: str = 'inter') -> None:
        """Set up the node and create the services."""
        super().__init__('camera_zoom_controller_service')
        self.logger = self.get_logger()

        self.controller = ZoomController()
        self.controller.set_speed(default_zoom_speed)
        for side in ('left', 'right'):
            self.controller.homing(side)
            self.controller.send_zoom_command(side, default_zoom_level)

        self.current_zoom_levels = {
            'left': default_zoom_level,
            'right': default_zoom_level,
        }
        self.current_zoom_speeds = {
            'left': default_zoom_speed,
            'right': default_zoom_speed,
        }

        self.get_zoom_level_service = self.create_service(
            GetCameraZoomLevel,
            'get_camera_zoom_level',
            self.get_zoom_level_callback,
        )
        self.logger.info(f'Launching "{self.get_zoom_level_service.srv_name}" service.')

        self.set_command_service = self.create_service(
            SetCameraZoomLevel,
            'set_camera_zoom_level',
            self.set_zoom_command_callback,
        )
        self.logger.info(f'Launching "{self.set_command_service.srv_name}" service.')

        self.get_speed_service = self.create_service(
            GetCameraZoomSpeed,
            'get_camera_zoom_speed',
            self.get_zoom_speed_callback,
        )

        self.set_speed_service = self.create_service(
            SetCameraZoomSpeed,
            'set_camera_zoom_speed',
            self.set_zoom_speed_callback,
        )
        self.logger.info(f'Launching "{self.set_speed_service.srv_name}" service.')

        self.logger.info('Node ready!')

    def get_zoom_level_callback(self,
                                request: GetCameraZoomLevel.Request,
                                response: GetCameraZoomLevel.Response,
                                ) -> GetCameraZoomLevel.Response:
        """Get the current camera zoom level."""
        response.zoom_level = self.current_zoom_levels[request.name]
        return response

    def set_zoom_command_callback(self,
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
            self.current_zoom_levels[request.name] = request.zoom_level

        else:
            self.logger.warning("Invalid command sent to zoom controller (must be in ('homing', 'in', 'out' or 'inter')).")
            response.success = False
            return response

        response.success = True
        return response

    def get_zoom_speed_callback(self,
                                request: GetCameraZoomSpeed.Request,
                                response: GetCameraZoomSpeed.Response,
                                ) -> GetCameraZoomSpeed.Response:
        """Get the current camera zoom speed."""
        response.zoom_level = self.current_zoom_speeds[request.name]
        return response

    def set_zoom_speed_callback(self,
                                request: SetCameraZoomSpeed.Request,
                                response: SetCameraZoomSpeed.Response,
                                ) -> SetCameraZoomSpeed.Response:
        """Handle set_camera_zoom_speed request."""
        if request.name not in ['left_eye', 'right_eye']:
            self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")
            response.success = False
            return response

        self.controller.set_speed(request.speed)
        self.current_zoom_speeds[request.name] = request.speed

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
