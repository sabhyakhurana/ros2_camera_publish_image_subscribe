#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Image Subscriber Example.

Revision History:
        2021-03-29 (Animesh): Baseline Software.

Example:
        $ ros2 run ros2_camera_publish execute

"""

#___Import Modules:
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = "/camera/image_raw"
QUEUE_SIZE = 1


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, TOPIC, 
                                                     self.listener_callback, QUEUE_SIZE)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("Image Received")
        
        #Convert the ROS Image message to a NumPy array
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        
        #Decode the image
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        
        if frame is not None:
            # Display the image
            cv2.imshow('Video Stream', frame)
            cv2.waitKey(1)
        else:
            self.get_logger().error('Failed to decode image')

def main(args=None):
    print("running here")
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print("running")
    main()

#                                                                              
# end of file
"""ANI717"""
