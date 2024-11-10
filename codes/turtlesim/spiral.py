#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.qos import QoSProfile

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        # Create publisher for turtle velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10)
        
        # Create subscriber for turtle pose
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        # Initialize pose variables
        self.pose = Pose()
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.move_turtle)
        
        # Spiral parameters
        self.angular_speed = 2.0  # rad/s
        self.initial_linear_speed = 1.0  # units/s
        self.spiral_growth_rate = 0.1  # How much to increase radius per rotation
        self.time_elapsed = 0.0
        
        self.get_logger().info('Turtle Spiral Controller has been started')

    def pose_callback(self, msg):
        """Callback function to update the turtle's pose"""
        self.pose = msg

    def move_turtle(self):
        """Timer callback to compute and publish velocity commands"""
        # Create Twist message for velocity commands
        vel_msg = Twist()
        
        # Calculate velocities for spiral motion
        # Linear velocity increases with time to create spiral effect
        self.time_elapsed += 0.1  # Increment time (based on timer period)
        
        # Linear velocity increases with time to create expanding spiral
        linear_speed = self.initial_linear_speed + self.spiral_growth_rate * self.time_elapsed
        
        # Set linear and angular velocities
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = self.angular_speed
        
        # Publish velocity commands
        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    
    turtle_controller = TurtleController()
    
    try:
        rclpy.spin(turtle_controller)
    except KeyboardInterrupt:
        pass
    
    # Clean up
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()