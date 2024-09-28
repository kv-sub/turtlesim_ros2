#!/usr/bin/env  python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounterNode(Node):

    def __init__(self):
        super().__init__("number_counter")
        self.counter_=0
        self.subscriber_ = self.create_subscription(Int64, "/number", self.callback_number, 10)
        self.get_logger().info("Number counter has been started")

        self.publishers_ = self.create_publisher(Int64, "/number_count", 10)
        
        self.reset_counter_service_ = self.create_service(SetBool,"reset_counter",self.callback_reset_counter)
        self.get_logger().info("Reset counter has been called")

    def callback_reset_counter(self,request,response):
        if request.data:
            self.counter_ = 0
            response.success = True
            response.message = "Counter has been reset"

        else:
            response.success = False
            response.message = "Counter has not been reset"

        return response 

    def timer_callback(self):
        self.counter_ += 1
        msg = Int64()
        msg.data = (self.counter_)
        self.publishers_.publish(msg)

    def callback_number(self, msg):
        self.create_timer(0.5,self.timer_callback)
        self.get_logger().info(str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown(node)

if __name__ == "__main__" :
    main()
