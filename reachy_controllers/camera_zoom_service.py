"""Service node to manage zoom for cameras."""
import time

import rclpy
from rclpy.node import Node

from reachy_msgs.srv import GetCameraZoomLevel, SetCameraZoomLevel
from reachy_msgs.srv import GetCameraZoomSpeed, SetCameraZoomSpeed
from reachy_msgs.srv import GetCameraZoomFocus, SetCameraZoomFocus
from reachy_msgs.srv import SetFocusState


class ZoomControllerService(Node):
    """Main node creating the zoom services for cameras."""

    def __init__(self, default_zoom_speed: int = 10000, default_zoom_level: str = 'inter') -> None:
        """Set up the node and create the services."""
        super().__init__('camera_zoom_controller_service')
        self.logger = self.get_logger()

        self.default_zoom_level = default_zoom_level
        self.current_zoom_info = {
            'left_eye': {
                'zoom': -1,
                'focus': -1,
                'speed': -1,
                'zoom_level': self.default_zoom_level,
            },
            'right_eye': {
                'zoom': -1,
                'focus': -1,
                'speed': -1,
                'zoom_level': self.default_zoom_level,
            },
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

        self.get_multiple_zoom_focus_service = self.create_service(
            GetCameraZoomFocus,
            'get_camera_zoom_focus',
            self.get_camera_zoom_focus_callback,
        )

        self.set_multiple_zoom_focus_service = self.create_service(
            SetCameraZoomFocus,
            'set_camera_zoom_focus',
            self.set_camera_zoom_focus_callback,
        )

        self.set_focus_state = self.create_client(
            SetFocusState,
            'set_focus_state',
        )

        self.logger.info('Node ready!')

    def get_zoom_level_callback(self,
                                request: GetCameraZoomLevel.Request,
                                response: GetCameraZoomLevel.Response,
                                ) -> GetCameraZoomLevel.Response:
        """Get the current camera zoom level."""
        if request.name not in ['left_eye', 'right_eye']:
            self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")
            return response
        response.zoom_level = self.current_zoom_info[request.name]['zoom_level']
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
        except KeyError:
            self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")
            response.success = False
            return response

        response.success = True
        return response

    def get_zoom_speed_callback(self,
                                request: GetCameraZoomSpeed.Request,
                                response: GetCameraZoomSpeed.Response,
                                ) -> GetCameraZoomSpeed.Response:
        """Get the current camera zoom speed."""
        if request.name not in ['left_eye', 'right_eye']:
            self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")
            return response

        response.speed = self.current_zoom_info[request.name]['speed']
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

        response.success = True
        return response

    def get_camera_zoom_focus_callback(self,
                                       request: GetCameraZoomFocus.Request,
                                       response: GetCameraZoomFocus.Response,
                                       ) -> GetCameraZoomFocus.Response:
        """Handle get_camera_zoom_focus callback."""
        response.left_focus = self.current_zoom_info['left_eye']['focus']
        response.left_zoom = self.current_zoom_info['left_eye']['zoom']
        response.right_focus = self.current_zoom_info['right_eye']['focus']
        response.right_zoom = self.current_zoom_info['right_eye']['zoom']
        return response

    def set_camera_zoom_focus_callback(self,
                                       request: SetCameraZoomFocus.Request,
                                       response: SetCameraZoomFocus.Response,
                                       ) -> SetCameraZoomFocus.Response:
        """Handle set_camera_zoom_focus callback."""
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
