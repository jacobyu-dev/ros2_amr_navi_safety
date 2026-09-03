import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── Package directories ─────────────────────────────────────────────────
    mir_description_dir = get_package_share_directory('mir_description')
    mir_gazebo_dir      = get_package_share_directory('mir_gazebo')
    ros_gz_sim_dir      = get_package_share_directory('ros_gz_sim')

    # ── Launch arguments ────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file   = LaunchConfiguration(
        'world',
        default=os.path.join(mir_gazebo_dir, 'worlds', 'maze.world')
    )
    headless     = LaunchConfiguration('headless', default='true')

    # ── GZ_SIM_RESOURCE_PATH: lets Gazebo resolve model:// mesh URIs ────────
    # When URDF→SDF conversion happens, "package://mir_description/..." becomes
    # "model://mir_description/...".  Pointing GZ_SIM_RESOURCE_PATH to the
    # parent of the installed share directory lets Gazebo find those files.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(mir_description_dir, '..')  # install/.../share/
    )

    # ── 1. Process Xacro → robot_description ───────────────────────────────
    urdf_xacro_path = os.path.join(mir_description_dir, 'urdf', 'mir.urdf.xacro')
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_xacro_path]), value_type=str
    )

    # ── 2. Robot State Publisher ────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }]
    )

    # ── 3. Gazebo Harmonic ──────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                PythonExpression(["'-r -s ' if '", headless, "' == 'true' else '-r '"]),
                world_file
            ],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ── 4. Spawn robot from /robot_description ──────────────────────────────
    # Spawn inside mapped free space near the maze centre (5m wall clearance).
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_mir',
        output='screen',
        arguments=[
            '-name',  'mir',
            '-topic', '/robot_description',
            '-x',     '-5.0',
            '-y',     '-6.0',
            '-z',     '0.15',
        ],
    )

    # ── 5. ros_gz_bridge ───────────────────────────────────────────────────
    # Direction notation:
    #   [   Gz → ROS  (sensor data flowing into ROS)
    #   ]   ROS → Gz  (commands flowing into Gazebo)
    #   @   bidirectional
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Sensor data  Gz → ROS
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Camera sensor data Gz → ROS
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth_image/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Odometry + TF + Joint States  Gz → ROS  (published by gz-sim-diff-drive-system & joint-state-publisher)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Drive command  ROS → Gz  (consumed by gz-sim-diff-drive-system)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Nav2 velocity smoother output also drives the robot
            '/cmd_vel_smoothed@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run Gazebo in headless mode (server only)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(mir_gazebo_dir, 'worlds', 'maze.world'),
            description='Full path to the Gazebo world file'),

        gz_resource_path,       # must come before gz_sim so env is set first
        gz_sim,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_entity]),
        bridge,
        # NOTE: mir_controller is NOT launched here.
        # In Gazebo Harmonic, gz-sim-diff-drive-system handles:
        #   • wheel actuation  (from /cmd_vel via bridge)
        #   • /odom publication (bridged to ROS)
        #   • odom→base_footprint TF
        # mir_controller is still used for real-hardware deployments via
        # src/mir_control/launch/mir_control.launch.py
    ])
