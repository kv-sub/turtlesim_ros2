#!/usr/bin/env  python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_no_oop")

    client = node.create_client(AddTwoInts,"add_two_ints")
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for Server Add two Ints ...")

    request = AddTwoInts.Request()
    request.a = 3
    request.b = 5

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(str(request.a) + "+" + str(request.b)+ "=" +str(response.sum))
    except Exception as e:
        node.get_logger().error("Service failed with the error %r" %(e,))


    rclpy.spin(node)
    rclpy.shutdown(node)

if __name__ == "__main__" :
    main()
