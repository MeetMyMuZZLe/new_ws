#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import transforms3d
import math

class CirclePathNode(Node):
    def __init__(self):
        super().__init__("circle_path")
        
        # Circle parameters
        self.radius = 2.5  # radius of the circle in meters
        self.center_x = 0.0  # x-coordinate of circle center
        self.center_y = 0.0  # y-coordinate of circle center
        self.angle = 0.0    # current angle in radians
        self.angle_increment = 0.1  # how much to increment angle each time
        
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscriber = self.create_subscription(Odometry, "odom", self.control_loop, 10)
        
        # Control parameters
        self.distance_threshold = 0.2
        self.angle_threshold = 0.1
        self.angular_speed = 0.4
        self.linear_speed = 0.3
        
        self.get_logger().info("Circle Path Controller Started")
    
    def get_current_target(self):
        # Calculate target point on circle
        target_x = self.center_x + self.radius * math.cos(self.angle)
        target_y = self.center_y + self.radius * math.sin(self.angle)
        return (target_x, target_y)
    
    def move_to_next_target(self):
        # Update angle for next target point on circle
        self.angle = (self.angle + self.angle_increment) % (2 * math.pi)
        target_x, target_y = self.get_current_target()
        self.get_logger().debug(f'Moving to point: ({round(target_x,2)}, {round(target_y,2)})')
        
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
    node = CirclePathNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()