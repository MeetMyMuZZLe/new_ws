#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class MyNode(Node): #MIDIFY NAME OF THE CLASS
    def __init__(self):
        super().__init__('sample')
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Congratulation for starting your Robot Operating Syatem Lab!!")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    