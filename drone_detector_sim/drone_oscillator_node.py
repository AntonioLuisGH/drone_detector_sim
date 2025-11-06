#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.time import Time

class DroneOscillatorNode(Node):
    """
    This node moves the drone up and down by publishing velocity commands.
    
    It publishes a Twist message to the topic '/model/target_drone/cmd_vel',
    which is the standard Gazebo (Ignition) topic for controlling a model's velocity.
    
    A bridge must be running to translate this ROS 2 message to a Gazebo message.
    """
    
    def __init__(self):
        super().__init__('drone_oscillator_node')
        
        # --- Parameters ---
        # Declare parameters for easy configuration
        self.declare_parameter('speed', 0.5)  # meters per second
        self.declare_parameter('duration', 3.0) # seconds in one direction
        
        # Get parameter values
        self._speed = self.get_parameter('speed').value
        self._duration_sec = self.get_parameter('duration').value
        
        self.get_logger().info(f'Starting drone oscillator: '
                             f'Speed: {self._speed} m/s, '
                             f'Duration: {self._duration_sec} s')

        # --- Publisher ---
        # We publish to the ROS 2 topic that the bridge will listen to.
        # By default, Gazebo expects velocity commands on '/model/<model_name>/cmd_vel'
        self._publisher = self.create_publisher(
            Twist,
            '/model/target_drone/cmd_vel',
            10)

        # --- State Variables ---
        self._direction = 1.0  # 1.0 for UP, -1.0 for DOWN
        self._start_time = None # Will be set on the first timer call
        
        # --- Timer ---
        # Create a timer to publish commands at 20 Hz (every 0.05 seconds)
        self._timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Node started. Waiting for simulation clock...')

    def timer_callback(self):
        """
        Called every 0.05 seconds. Calculates time elapsed and publishes
        the appropriate velocity command.
        """
        current_time = self.get_clock().now()
        
        # Wait for the simulation clock to start
        if current_time.nanoseconds == 0:
            self.get_logger().info('Waiting for valid simulation time...', once=True)
            return

        # Initialize start time on the first valid run
        if self._start_time is None:
            self._start_time = current_time
            self.get_logger().info('Simulation clock active. Starting movement.')
        
        # Calculate how long we've been moving in the current direction
        elapsed_duration = (current_time - self._start_time).nanoseconds / 1e9 # Convert to seconds
        
        # Check if it's time to switch direction
        if elapsed_duration > self._duration_sec:
            self._direction *= -1.0  # Flip direction
            self._start_time = current_time # Reset the timer
            
            direction_str = "UP" if self._direction > 0 else "DOWN"
            self.get_logger().info(f'Switching direction: Now moving {direction_str}')

        # --- Create and Publish Message ---
        twist_msg = Twist()
        
        # We only set the linear 'z' velocity for up/down movement
        twist_msg.linear.z = self._speed * self._direction
        
        # Publish the message
        self._publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = None # Initialize node to None
    try:
        node = DroneOscillatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('KeyboardInterrupt received, shutting down.')
        else:
            print('KeyboardInterrupt received before node initialization.')
    except Exception as e:
        # Log any other exceptions that might occur
        if node:
            node.get_logger().error(f"Unhandled exception in spin: {e}")
        else:
            print(f"Error during node initialization: {e}")
    finally:
        # Check if node and its publisher were successfully created before using them
        if node and hasattr(node, '_publisher'):
            # Stop the drone before shutting down
            stop_msg = Twist()
            node._publisher.publish(stop_msg)
            node.get_logger().info('Stopping drone and shutting down.')
        elif node:
            node.get_logger().warn('Shutting down, but publisher was not initialized.')
        else:
            print("Shutting down after initialization error.")
            
        if node:
            node.destroy_node()
        
        # Only shutdown if rclpy is still initialized
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()  