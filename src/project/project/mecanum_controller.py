#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from math import cos, sin

class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')
        
        # Controller parameters
        self.wheel_separation_w = 1.06067  # Width separation between wheels
        self.wheel_separation_l = 1.06667  # Length separation between wheels
        self.wheel_diameter = 0.59918     # Wheel diameter
        self.max_wheel_torque = 30.0      # Maximum wheel torque
        self.max_wheel_accel = 30.0       # Maximum wheel acceleration
        
        # Joint names
        self.joint_names = [
            'rear_left_wheel_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_right_wheel_joint'
        ]
        
        # Initialize velocities
        self.desired_wheel_speeds = [0.0] * 4
        self.current_wheel_speeds = [0.0] * 4
        
        # Create publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )
        
        # Create timer for control loop
        self.control_timer = self.create_timer(
            1/30.0,  # 30 Hz control rate
            self.control_callback
        )
        
        # Initialize messages
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.position = [0.0] * 4
        self.joint_state_msg.velocity = [0.0] * 4
        self.joint_state_msg.effort = [0.0] * 4
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_footprint'
        
        # Store last command and time
        self.last_cmd = Twist()
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands."""
        self.last_cmd = msg
        self.update_wheel_velocities(msg.linear.x, msg.linear.y, msg.angular.z)
        
    def update_wheel_velocities(self, vx, vy, va):
        """Calculate desired wheel velocities using mecanum drive kinematics."""
        # Factor for converting linear/angular velocity to wheel velocity
        wheel_radius = self.wheel_diameter / 2.0
        
        # Calculate wheel velocities using mecanum drive inverse kinematics
        self.desired_wheel_speeds[0] = (vx + vy - va * (self.wheel_separation_w + self.wheel_separation_l) / 2.0) / wheel_radius  # Left Rear
        self.desired_wheel_speeds[1] = (vx - vy - va * (self.wheel_separation_w + self.wheel_separation_l) / 2.0) / wheel_radius  # Left Front
        self.desired_wheel_speeds[2] = -(vx + vy + va * (self.wheel_separation_w + self.wheel_separation_l) / 2.0) / wheel_radius  # Right Front
        self.desired_wheel_speeds[3] = -(vx - vy + va * (self.wheel_separation_w + self.wheel_separation_l) / 2.0) / wheel_radius  # Right Rear

    def calculate_odometry(self, dt):
        """Calculate robot odometry from wheel velocities."""
        wheel_radius = self.wheel_diameter / 2.0
        l = 1.0 / (2 * (self.wheel_separation_w + self.wheel_separation_l))
        
        # Calculate robot velocity in robot frame
        vel_x = sum([-speed * wheel_radius for speed in self.current_wheel_speeds]) / 4.0
        vel_y = (self.current_wheel_speeds[0] - self.current_wheel_speeds[1] + 
                 self.current_wheel_speeds[2] - self.current_wheel_speeds[3]) * wheel_radius / 4.0
        vel_th = (-self.current_wheel_speeds[0] - self.current_wheel_speeds[1] + 
                  self.current_wheel_speeds[2] + self.current_wheel_speeds[3]) * l * wheel_radius / 4.0
        
        # Update odometry message
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.twist.twist.linear.x = vel_x
        self.odom_msg.twist.twist.linear.y = vel_y
        self.odom_msg.twist.twist.angular.z = vel_th
        
        # Set covariance
        self.odom_msg.pose.covariance[0] = 0.00001  # x
        self.odom_msg.pose.covariance[7] = 0.00001  # y
        self.odom_msg.pose.covariance[35] = 0.001   # yaw
        
        self.odom_msg.twist.covariance = self.odom_msg.pose.covariance

    def control_callback(self):
        """Main control loop."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Update wheel speeds with acceleration limits
        for i in range(4):
            speed_diff = self.desired_wheel_speeds[i] - self.current_wheel_speeds[i]
            if abs(speed_diff) > self.max_wheel_accel * dt:
                self.current_wheel_speeds[i] += np.sign(speed_diff) * self.max_wheel_accel * dt
            else:
                self.current_wheel_speeds[i] = self.desired_wheel_speeds[i]
        
        # Publish joint states
        self.joint_state_msg.header.stamp = current_time.to_msg()
        self.joint_state_msg.velocity = self.current_wheel_speeds
        self.joint_state_pub.publish(self.joint_state_msg)
        
        # Calculate and publish odometry
        self.calculate_odometry(dt)
        self.odom_pub.publish(self.odom_msg)

def main():
    rclpy.init()
    controller = MecanumController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()