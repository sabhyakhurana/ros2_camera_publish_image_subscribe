import os
import cv2
import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

import threading



#___Global Variables:
DEVICE_INDEX = 0 # specifies which camera
TOPIC = '/camera/image_raw'
QUEUE_SIZE = 1
PERIOD = 0.01  # seconds
QUALITY = 25

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, TOPIC, QUEUE_SIZE)

        self.capture = cv2.VideoCapture(0)  # Adjust camera index if needed

        self.timer = self.create_timer(PERIOD, self.timer_callback)  # Adjust timer interval for desired FPS
        
        self.i = 0
        self.lock = threading.Lock()

    def timer_callback(self):
        """Timer Callback Function
        
        This method captures images and publishes required data in ros 2 topic.
        """
        with self.lock:
            if not self.capture.isOpened():
                self.get_logger().error('Unable to open camera')
                return

            ret, frame = self.capture.read()
            frame = cv2.flip(frame, -1)

            if not ret:
                self.get_logger().error('Failed to capture image')
                return


            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), QUALITY]
            ret, buffer = cv2.imencode('.jpeg', frame, encode_param)

            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "jpeg"
            msg.is_bigendian = False
            msg.step = len(buffer)
            msg.data = buffer.tobytes()

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published image {self.i}')
            self.i += 1

    def cleanup(self):
        self.capture.release()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()