#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import transforms3d
import math
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error
        self.last_time = current_time
        
        return output

class GotoGoalNode(Node):
    def __init__(self):
        super().__init__("move_robot")
        self.target_x = 2
        self.target_y = 2
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscriber = self.create_subscription(Odometry, "odom", self.control_loop, 10)
        
        # PID controllers for linear and angular motion
        self.linear_pid = PIDController(0.5, 0.01, 0.1)
        self.angular_pid = PIDController(0.8, 0.01, 0.2)
        
    def control_loop(self, msg):
        dist_x = self.target_x - msg.pose.pose.position.x
        dist_y = self.target_y - msg.pose.pose.position.y
        print(f'Current position: {msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}')
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        print(f'Distance: {distance:.3f}')
        
        goal_theta = math.atan2(dist_y, dist_x)
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = transforms3d.euler.quat2euler([quat.w, quat.x, quat.y, quat.z])
        angle_diff = math.atan2(math.sin(goal_theta - yaw), math.cos(goal_theta - yaw))
        
        print(f'Yaw: {yaw:.2f}')
        print(f'Target angle: {goal_theta:.2f}')
        print(f'Orientation difference: {angle_diff:.2f}')
        
        vel = Twist()
        
        # Compute PID outputs
        linear_output = self.linear_pid.compute(distance)
        angular_output = self.angular_pid.compute(angle_diff)
        
        # Apply PID outputs to velocity commands
        vel.linear.x = max(min(linear_output, 0.5), -0.5)  # Limit max speed to 0.5 m/s
        vel.angular.z = max(min(angular_output, 1.0), -1.0)  # Limit max angular speed to 1.0 rad/s
        
        # If we're close to the goal, stop
        if distance < 0.1 and abs(angle_diff) < 0.1:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            print("Goal reached!")
        
        print(f'Command velocity: Linear: {vel.linear.x:.2f}, Angular: {vel.angular.z:.2f}')
        self.publisher.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = GotoGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()