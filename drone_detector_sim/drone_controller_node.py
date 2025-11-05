#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

# --- Configuration ---
# This topic name MUST match the bridge and SDF file
NODE_NAME = 'drone_controller'
PUBLISH_TOPIC = '/drone/cmd_vel'
PUBLISH_RATE_HZ = 10.0
TARGET_UPDATE_RATE_HZ = 0.33 # Update target every ~3 seconds

class DroneControllerNode(Node):
    """
    This node publishes random velocity commands to move the drone around.
    """
    def __init__(self):
        super().__init__(NODE_NAME)
        
        # Create the publisher
        self.publisher_ = self.create_publisher(Twist, PUBLISH_TOPIC, 10)
        
        # Timer to publish commands at a steady rate
        self.publish_timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ, 
            self.publish_callback
        )
        
        # Timer to update the random target velocity
        self.update_target_timer = self.create_timer(
            1.0 / TARGET_UPDATE_RATE_HZ,
            self.update_target_callback
        )
        
        self.current_twist = Twist()
        
        self.get_logger().info(
            f'Drone Controller started. Publishing random commands to {PUBLISH_TOPIC}'
        )
        
        # Set an initial target
        self.update_target_callback()

    def update_target_callback(self):
        """
        Sets a new random target velocity.
        """
        # Set a random linear speed between 0.5 and 2.5 m/s
        self.current_twist.linear.x = random.uniform(0.5, 2.5)
        self.current_twist.linear.y = 0.0 # No strafing
        self.current_twist.linear.z = 0.0 # No up/down
        
        # Set a random angular speed between -1.0 and 1.0 rad/s
        self.current_twist.angular.x = 0.0
        self.current_twist.angular.y = 0.0
        self.current_twist.angular.z = random.uniform(-1.0, 1.0)
        
        self.get_logger().info(
            f'New target: linear.x={self.current_twist.linear.x:.2f}, '
            f'angular.z={self.current_twist.angular.z:.2f}'
        )

    def publish_callback(self):
        """
        Publishes the current target velocity.
        """
        self.publisher_.publish(self.current_twist)

def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()