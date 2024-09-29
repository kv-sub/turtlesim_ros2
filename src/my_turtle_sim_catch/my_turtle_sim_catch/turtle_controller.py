#!/usr/bin/env  python3
from re import T
import rclpy
from rclpy.node import Node
from functools import partial

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

import math

class TurtleController(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("catch_closest_turtle_", True)


        self.turtle_to_catch = None
        self.catch_closest_turtle_ = self.get_parameter("catch_closest_turtle_").value
        self.pose_ = None
        self.pose_subscriber_ = self.create_subscription(Pose,"/turtle1/pose",self.callback_turtle_pose, 10)
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray,"alive_turtle",self.callback_alive_turtles,10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist,"/turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.01, self.control_loop)


    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_:
                closest_turtle_ = None
                closest_turtle_distance_ = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)
                    if closest_turtle_distance_ == None or closest_turtle_distance_ > distance :
                        closest_turtle_distance_ = distance
                        closest_turtle_ = turtle

                self.turtle_to_catch = closest_turtle_

            else:
                self.turtle_to_catch = msg.turtles[0]

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch == None:
            return
        dist_x = self.turtle_to_catch.x - self.pose_.x
        dist_y = self.turtle_to_catch.y - self.pose_.y
        distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)

        msg = Twist()

        if distance > 0.5 :
            # position
            msg.linear.x = 2*distance

            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi :
                diff -= 2*math.pi
            elif diff < -math.pi :
                diff += 2*math.pi
            
            msg.angular.z = 6*diff

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0 
            self.call_catch_turtle_server(self.turtle_to_catch.name)
            self.turtle_to_catch = None

        self.cmd_vel_publisher_.publish(msg)

    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle,"catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server ...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_catch_turtle, turtle_name = turtle_name))

    def callback_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name) + " could not be caught")
        except Exception as e:
            self.get_logger().error("Service failed with the error %r" %(e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown(node)

if __name__ == "__main__" :
    main()
