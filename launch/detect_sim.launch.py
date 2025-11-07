import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- File Paths ---
    pkg_share = get_package_share_directory('drone_detector_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    world_file = os.path.join(pkg_share, 'worlds', 'detector_world.sdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'config.rviz')
    
    # --- Environment Variable ---
    # Use os.path.expanduser to handle the '~' correctly.
    rgl_patterns_dir = os.path.expanduser('~/simulations/RGLGazeboPlugin-0.2.0-fortress/lidar_patterns')
    
    set_env_var = SetEnvironmentVariable(
        name='RGL_PATTERNS_DIR',
        value=rgl_patterns_dir
    )

    # --- Gazebo (The *Correct* ROS 2 Way) ---
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # Pass the world file as an argument to the included launch file
        # We no longer need to force use_sim_time, so we remove the RosClock
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

   # Lidar Bridge (GZ -> ROS)
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar/avia@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        output='screen'
    )

    # Pose Bridge (GZ -> ROS)
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[f'/world/drone_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        output='screen'
    )

    # Manually publish the static transform for the LiDAR
    # Pose is from SDF: <pose>0 0 0.5 0 0 0</pose>
    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.5', '0', '0', '0', 'drone_world', 'lidar_link']
    )

    # --- Nodes ---
    
    # Ground Truth BBox Node
    ground_truth_bbox_node = Node(
        package='drone_detector_sim',
        executable='ground_truth_bbox_node',
        output='screen'
    )
    
    # RViz
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # --- Launch Description ---
    return LaunchDescription([
        set_env_var,
        start_gazebo,
        lidar_bridge,
        pose_bridge,
        static_lidar_tf,
        ground_truth_bbox_node,
        start_rviz,
    ])