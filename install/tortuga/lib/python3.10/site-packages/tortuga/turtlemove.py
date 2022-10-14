#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

class Minode(Node):

    def __init__(self):
        
        super().__init__('turtlemove')
        self.publisher=self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        vel=Twist()

        vel.linear.x=6.0
        vel.linear.y=0.0
        vel.linear.z=0.0

        vel.angular.x=0.0
        vel.angular.y=0.0
        vel.angular.z=2.5

        self.publisher.publish(vel)




def main():
    try:
        rclpy.init()
        node = Minode()
        rclpy.spin(node)

    finally:
        rclpy.shutdown


if __name__ == '__main__':
    main()