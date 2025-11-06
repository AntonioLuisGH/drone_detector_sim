#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

# --- Configuration ---
WORLD_NAME = 'drone_world'
DRONE_MODEL_NAME = 'target_drone'
DRONE_SIZE_X = 0.47
DRONE_SIZE_Y = 0.98
DRONE_SIZE_Z = 0.15

class GroundTruthBboxPublisher(Node):
    """
    This node subscribes to Gazebo's ground truth pose topic
    and publishes a bounding box Marker for RViz.
    """
    def __init__(self):
        super().__init__('ground_truth_bbox_publisher')
        self.get_logger().info('Ground Truth BBox Node started, using simulation time.')

        gz_pose_topic = f'/world/{WORLD_NAME}/pose/info'

        self.tf_sub = self.create_subscription(
            TFMessage,
            gz_pose_topic,
            self.pose_callback,
            10)

        self.marker_pub = self.create_publisher(
            Marker,
            '/drone_bbox',
            10)

    def pose_callback(self, msg):
        current_time = self.get_clock().now().to_msg()

        # Wait for the /clock to start
        if current_time.sec == 0 and current_time.nanosec == 0:
            # No need to log here, the TF node is already logging
            return

        for transform in msg.transforms:
            if transform.child_frame_id == DRONE_MODEL_NAME:
                marker = Marker()
                marker.header.frame_id = "drone_world"
                marker.header.stamp = current_time
                marker.ns = "drone"
                marker.id = 0
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                marker.pose.position.x = transform.transform.translation.x
                marker.pose.position.y = transform.transform.translation.y
                marker.pose.position.z = transform.transform.translation.z
                marker.pose.orientation = transform.transform.rotation
                
                marker.scale.x = DRONE_SIZE_X
                marker.scale.y = DRONE_SIZE_Y
                marker.scale.z = DRONE_SIZE_Z
                
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.5
                
                self.marker_pub.publish(marker)
                # Found the drone, no need to keep looping
                break

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthBboxPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()