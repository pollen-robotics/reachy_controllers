'''Service node to manage zoom services.'''
import rclpy
from rclpy.node import Node

from reachy_msgs.srv import ZoomCommand, SetZoomSpeed

from optical_zoom.zoom_piloting import ZoomController


class ZoomControllerService(Node):
    '''Main node creating zoom services.'''
    def __init__(self):
        super().__init__('zoom_controller_service')
        self.controller = ZoomController()
        self.command_service = self.create_service(
            ZoomCommand,
            'zoom_command',
            self.command_callback
            )
        self.speed_service = self.create_service(
            SetZoomSpeed,
            'zoom_speed',
            self.speed_callback
            )
        self.logger = self.get_logger()
        self.logger.info('Initialized ZoomControllerService.')

    def command_callback(self, request, response):
        '''
        request:
            - string side
            - string zoom_command
        response:
            None
        '''
        if request.zoom_command == 'homing':
            self.controller.homing(request.side)
            return response
        self.controller.send_zoom_command(request.side, request.zoom_command)
        return response

    def speed_callback(self, request, response):
        '''
        request:
            - int16 speed
        response:
            None
        '''
        self.controller.set_speed(request.speed)
        return response


def main(args=None):
    '''Main.'''
    rclpy.init(args=args)

    orb_kin_service = ZoomControllerService()

    rclpy.spin(orb_kin_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
