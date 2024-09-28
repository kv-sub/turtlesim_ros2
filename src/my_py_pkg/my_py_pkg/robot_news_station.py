#!/usr/bin/env  python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String


class RobotNewsStationNode(Node):

    def __init__(self):
        super().__init__("robot_news_station")
        self.declare_parameter("robot_name", "CP3O")

        self.robot_name_ = self.get_parameter("robot_name").value
        self.publishers_ = self.create_publisher(String, "robot_news", 10)
        self.timers_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News has been started")

    def publish_news(self):

        msg = String()
        msg.data = (
            "Hi !! This is " + str(self.robot_name_) + " from Robot News Station."
        )
        self.publishers_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown(node)


if __name__ == "__main__":
    main()
