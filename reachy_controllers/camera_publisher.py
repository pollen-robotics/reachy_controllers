"""
Camera Node.

- publish /camera at the specified rate (default: 100Hz)

"""

import rclpy
from rclpy.node import Node

import cv2 as cv
from sensor_msgs.msg._compressed_image import CompressedImage
from cv_bridge import CvBridge


class CameraPublisher(Node):
    """Camera Publisher class."""

    def __init__(self, size: tuple = (1280, 720), rate: float = 100.0, fps: float = 30.0) -> None:
        """Connect to both cameras, initialize the publishers."""
        super().__init__('camera_publisher')

        self.image_left = CompressedImage()
        self.cap_left = cv.VideoCapture()
        self.cap_left.open(0)

        self.cap_left.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        self.cap_left.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap_left.set(cv.CAP_PROP_FPS, rate)
        self.cap_left.set(cv.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap_left.set(cv.CAP_PROP_FRAME_HEIGHT, size[1])

        self.image_right = CompressedImage()
        self.cap_right = cv.VideoCapture()
        self.cap_right.open(4)

        self.cap_right.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        self.cap_right.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap_right.set(cv.CAP_PROP_FPS, rate)
        self.cap_right.set(cv.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap_right.set(cv.CAP_PROP_FRAME_HEIGHT, size[1])

        self.clock = self.get_clock()
        self.camera_publisher_left = self.create_publisher(CompressedImage, 'left_image', 1)
        self.camera_publisher_right = self.create_publisher(CompressedImage, 'right_image', 1)
        self.publish_timer = self.create_timer(timer_period_sec=1/rate, callback=self.publish_msg)

        self.bridge = CvBridge()

    def publish_msg(self) -> None:
        """Read both images, convert each into an Image ROS msg and publish it."""
        _, img_left = self.cap_left.read()
        _, img_right = self.cap_right.read()
        self.img_msg_left = self.bridge.cv2_to_compressed_imgmsg(img_left)
        self.img_msg_right = self.bridge.cv2_to_compressed_imgmsg(img_right)
        self.camera_publisher_left.publish(self.img_msg_left)
        self.camera_publisher_right.publish(self.img_msg_right)


def main() -> None:
    """Run Camera publisher main loop."""
    rclpy.init()

    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()