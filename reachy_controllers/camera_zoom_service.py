"""Service node to manage zoom for cameras."""
import numpy as np

import rclpy
from rclpy.node import Node

from reachy_msgs.srv import GetCameraZoomLevel, SetCameraZoomLevel
from reachy_msgs.srv import GetCameraZoomSpeed, SetCameraZoomSpeed
from reachy_msgs.srv import GetCameraFocusZoom, SetCameraFocusZoom
from reachy_msgs.srv import Set2CamerasFocus, Set2CamerasZoom, Set2CamerasZoomLevel

from zoom_kurokesu import ZoomController


class ZoomControllerService(Node):
    """Main node creating the zoom services for cameras."""

    def __init__(self, default_zoom_speed: int = 10000, default_zoom_level: str = 'inter') -> None:
        """Set up the node and create the services."""
        super().__init__('camera_zoom_controller_service')
        self.logger = self.get_logger()

        self.controller = ZoomController()
        self.controller.set_speed(default_zoom_speed)
        for side in ('left', 'right'):
            self.controller.homing(side)
            self.controller.send_zoom_command(side, default_zoom_level)

        self.current_zoom_levels = {
            'left_eye': default_zoom_level,
            'right_eye': default_zoom_level,
        }
        self.current_zoom_speeds = {
            'left_eye': default_zoom_speed,
            'right_eye': default_zoom_speed,
        }

        self.current_focus = {
            'left_eye': self.controller.zoom_pos["left"][default_zoom_level]['focus'],
            'right_eye': self.controller.zoom_pos["right"][default_zoom_level]['focus'],
        }
        self.current_zoom = {
            'left_eye': self.controller.zoom_pos["left"][default_zoom_level]['zoom'],
            'right_eye': self.controller.zoom_pos["right"][default_zoom_level]['zoom'],
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

        self.get_zoom_focus_service = self.create_service(
            GetCameraFocusZoom,
            'get_camera_focus_zoom',
            self.get_camera_focus_zoom_callback,
        )
        self.logger.info(f'Launching "{self.get_zoom_focus_service.srv_name}" service.')

        self.set_zoom_focus_service = self.create_service(
            SetCameraFocusZoom,
            'set_camera_focus_zoom',
            self.set_camera_focus_zoom_callback,
        )
        self.logger.info(f'Launching "{self.set_zoom_focus_service.srv_name}" service.')

        self.set_focus_2_cameras_service = self.create_service(
            Set2CamerasFocus,
            'set_2_cameras_focus',
            self.set_2_cameras_focus_callback,
        )
        self.logger.info(f'Launching "{self.set_focus_2_cameras_service.srv_name}" service.')

        self.set_zoom_2_cameras_service = self.create_service(
            Set2CamerasZoom,
            'set_2_cameras_zoom',
            self.set_2_cameras_zoom_callback,
        )
        self.logger.info(f'Launching "{self.set_zoom_2_cameras_service.srv_name}" service.')

        self.set_zoom_2_cameras_level_service = self.create_service(
            Set2CamerasZoomLevel,
            'set_2_cameras_zoom_level',
            self.set_2_cameras_zoom_level_callback,
        )
        self.logger.info(f'Launching "{self.set_zoom_2_cameras_level_service.srv_name}" service.')

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
            self.current_zoom_levels[request.name] = 'zero'

        elif request.zoom_level in ('in', 'out', 'inter'):
            self.controller.send_zoom_command(eye_side, request.zoom_level)
            self.current_zoom_levels[request.name] = request.zoom_level
            self.current_zoom[request.name] = int(self.controller.zoom_pos[eye_side][request.zoom_level]['zoom'])

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
        response.speed = self.current_zoom_speeds[request.name]
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

    def get_camera_focus_zoom_callback(self,
                                       request: GetCameraFocusZoom.Request,
                                       response: GetCameraFocusZoom.Response,
                                       ) -> GetCameraFocusZoom.Response:
        """Get the current camera focus and zoom value."""
        try:
            response.focus = self.current_focus[request.name]
            response.zoom = self.current_zoom[request.name]
        except KeyError:
            self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")

        return response

    def set_camera_focus_zoom_callback(self,
                                       request: SetCameraFocusZoom.Request,
                                       response: SetCameraFocusZoom.Response,
                                       ) -> SetCameraFocusZoom.Response:
        """Handle set_camera_zoom_speed request."""
        try:
            eye_side = {
                'left_eye': 'left',
                'right_eye': 'right',
            }[request.name]
        except KeyError:
            self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")
            response.success = False
            return response

        self.controller._send_custom_command(eye_side, request.zoom, request.focus)
        self.current_focus[request.name] = int(np.clip(request.focus, 0, 500))
        self.current_zoom[request.name] = int(np.clip(request.zoom, 0, 600))

        response.success = True
        return response

    def set_2_cameras_focus_callback(self,
                                     request: Set2CamerasFocus.Request,
                                     response: Set2CamerasFocus.Response,
                                     ) -> Set2CamerasFocus.Response:
        """Handle set_focus_2_cameras_service request."""
        try:
            left_focus = int(request.left_focus)
            right_focus = int(request.right_focus)

        except KeyError:
            self.logger.warning("Invalid focus value sent to zoom controller (must be betwin 0 and 500).")
            response.success = False
            return response

        self.controller.send_custom_focus_two_cameras(left_focus, right_focus)
        self.current_focus["left_eye"] = int(np.clip(request.left_focus, 0, 500))
        self.current_focus["right_eye"] = int(np.clip(request.right_focus, 0, 500))

        response.success = True
        return response

    def set_2_cameras_zoom_callback(self,
                                    request: Set2CamerasZoom.Request,
                                    response: Set2CamerasZoom.Response,
                                    ) -> Set2CamerasZoom.Response:
        """Handle set_zoom_2_cameras_service request."""
        try:
            left_zoom = int(request.left_zoom)
            right_zoom = int(request.right_zoom)
        except KeyError:
            self.logger.warning("Invalid focus value sent to zoom controller (must be betwin 0 and 500).")
            response.success = False
            return response

        self.controller.send_custom_zoom_two_cameras(left_zoom, right_zoom)
        self.current_zoom["left_eye"] = int(np.clip(request.left_zoom, 0, 500))
        self.current_zoom["right_eye"] = int(np.clip(request.right_zoom, 0, 500))

        response.success = True
        return response

    def set_2_cameras_zoom_level_callback(self,
                                          request: Set2CamerasZoomLevel.Request,
                                          response: Set2CamerasZoomLevel.Response,
                                          ) -> Set2CamerasZoomLevel.Response:
        """Handle set_2_cameras_zoom_level request."""
        if request.left_zoom in ('in', 'out', 'inter') and request.right_zoom in ('in', 'out', 'inter'):
            self.controller.send_zoom_command_two_cameras(request.left_zoom, request.right_zoom)
            self.current_zoom_levels["left_eye"] = request.left_zoom
            self.current_zoom["left_eye"] = int(self.controller.zoom_pos["left"][request.left_zoom]['zoom'])
            self.current_zoom_levels["right_eye"] = request.right_zoom
            self.current_zoom["right_eye"] = int(self.controller.zoom_pos["right"][request.right_zoom]['zoom'])

        else:
            self.logger.warning("Invalid command sent to zoom controller (must be in ('homing', 'in', 'out' or 'inter')).")
            response.success = False
            return response

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
