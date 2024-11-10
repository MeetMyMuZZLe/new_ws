#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import transforms3d
import math

class TrianglePathNode(Node):
    def __init__(self):
        super().__init__("triangle_path")
        
        # Define the triangle vertices
        self.triangle_points = [
            (0.0, 0.0),    # Starting point
            (-5.0, 0.0),    # First vertex
            (-2.5, 3.0),    # Second vertex
            (0.0, 0.0)     # Return to start
        ]
        
        self.current_target = 1  # Index of current target point
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscriber = self.create_subscription(Odometry, "odom", self.control_loop, 10)
        
        # Control parameters
        self.distance_threshold = 0.2
        self.angle_threshold = 0.1
        self.angular_speed = 0.4
        self.linear_speed = 0.3
        
        self.get_logger().info("Triangle Path Controller Started")
        
    def get_current_target(self):
        return self.triangle_points[self.current_target]
    
    def move_to_next_target(self):
        self.current_target = (self.current_target + 1) % len(self.triangle_points)
        target = self.get_current_target()
        self.get_logger().info(f'Moving to point {self.current_target}: ({target[0]}, {target[1]})')
        
    def control_loop(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        target_x, target_y = self.get_current_target()
        
        # Calculate distance to target
        dist_x = target_x - current_x
        dist_y = target_y - current_y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        
        # Calculate angle to target
        goal_theta = math.atan2(dist_y, dist_x)
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = transforms3d.euler.quat2euler([quat.w, quat.x, quat.y, quat.z])
        
        # Calculate angle difference
        diff = math.pi - yaw + goal_theta
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
            
        # Log current status
        self.get_logger().debug(f'Position: ({round(current_x,2)}, {round(current_y,2)})')
        self.get_logger().debug(f'Distance to target: {round(distance,2)}')
        self.get_logger().debug(f'Angle to target: {round(diff,2)}')
        
        # Create and publish velocity command
        vel = Twist()
        
        if distance < self.distance_threshold:
            # Reached current target, move to next point
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.publisher.publish(vel)
            self.move_to_next_target()
            
        elif abs(diff) > self.angle_threshold:
            # Rotate to face target
            vel.linear.x = 0.0
            vel.angular.z = self.angular_speed * diff
            
        else:
            # Move towards target
            vel.linear.x = self.linear_speed * distance
            vel.angular.z = 0.0
            
        self.publisher.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = TrianglePathNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()