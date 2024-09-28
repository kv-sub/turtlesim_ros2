#!/usr/bin/env  python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64

class NumberPublisherNode(Node):

    def __init__(self):
        super().__init__("number_publisher")

        self.publishers_ = self.create_publisher(Int64,"/number",10)
        self.timers_ = self.create_timer(0.5,self.publish_number)
        self.get_logger().info("Number has been started")

    def publish_number(self):
        
        msg = Int64()
        msg.data = (20)
        self.publishers_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown(node)

if __name__ == "__main__" :
    main()
