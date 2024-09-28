#!/usr/bin/env  python3
import rclpy
from rclpy.node import Node
from functools import partial

from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):

    def __init__(self):
        super().__init__("add_two_ints_client")
        self.call_add_two_ints_server(6,7)
        self.call_add_two_ints_server(6,5)
        self.call_add_two_ints_server(3,7)


    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts,"add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Add two Ints ...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_add_two_ints, a=a, b=b))

    def callback_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(str(a) + "+" + str(b)+ "=" +str(response.sum))
        except Exception as e:
            self.get_logger().error("Service failed with the error %r" %(e,))


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown(node)

if __name__ == "__main__" :
    main()
