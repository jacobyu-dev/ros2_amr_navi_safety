"""Launch one self-contained AMR world and a MiR100 via Gazebo Harmonic.

No Nav2, AMCL, SLAM, mission, or safety node is included here by design.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


BASE_WORLD_FILES = {
    'empty': ('empty', 'empty.sdf'),
    'maze': ('mir_maze', 'maze.sdf'),
    'corridor': ('corridor', 'corridor.sdf'),
    'bookstore': ('bookstore', 'bookstore.sdf'),
    'warehouse': ('warehouse', 'warehouse.sdf'),
    'warehouse_detailed': ('warehouse_detailed', 'warehouse_detailed.sdf'),
}
BASE_SPAWN_POSES = {
    'empty': ('0.0', '0.0'),
    'maze': ('-5.0', '-6.0'),
    'corridor': ('0.0', '0.0'),
    'bookstore': ('12.0', '-9.0'),
    'warehouse': ('0.0', '0.0'),
    'warehouse_detailed': ('0.0', '0.0'),
}
BASE_GZ_WORLD_NAMES = {
    'empty': 'amr_empty',
    'maze': 'maze_world',
    'corridor': 'corridor',
    'bookstore': 'amr_bookstore',
    'warehouse': 'amr_warehouse',
    'warehouse_detailed': 'amr_warehouse_detailed',
}

VARIANT_COUNTS = (0, 3, 10)
WORLD_FILES = dict(BASE_WORLD_FILES)
WORLD_BASE_NAMES = {name: name for name in BASE_WORLD_FILES}
FIXED_OBSTACLE_COUNTS = {}
SPAWN_POSES = dict(BASE_SPAWN_POSES)
for base_name, file_spec in BASE_WORLD_FILES.items():
    for obstacle_count in VARIANT_COUNTS:
        variant_name = f'{base_name}_{obstacle_count}'
        WORLD_FILES[variant_name] = file_spec
        WORLD_BASE_NAMES[variant_name] = base_name
        FIXED_OBSTACLE_COUNTS[variant_name] = obstacle_count
        SPAWN_POSES[variant_name] = BASE_SPAWN_POSES[base_name]

# Every worker follows the same local -3 m -> +3 m -> -3 m trajectory from
# worker.sdf. X/Y/Yaw rotate and translate that segment into collision-free
# aisles. The first three entries are also used by each *_3 scenario.
WORKER_SPAWNS_BY_WORLD = {
    'empty': [
        ('0.0', '0.0', '0.0'),
        ('0.0', '0.0', '1.5708'),
        ('0.0', '3.0', '0.0'),
        ('-6.0', '0.0', '1.5708'),
        ('6.0', '0.0', '1.5708'),
        ('0.0', '-6.0', '0.0'),
        ('0.0', '6.0', '0.0'),
        ('-3.0', '0.0', '1.5708'),
        ('3.0', '0.0', '1.5708'),
        ('0.0', '-3.0', '0.0'),
    ],
    'maze': [
        ('5.0', '0.0', '0.0'),
        ('5.0', '3.0', '1.5708'),
        ('-5.0', '-6.0', '0.0'),
        ('-5.0', '-3.0', '1.5708'),
        ('-5.0', '0.0', '0.0'),
        ('-5.0', '6.0', '0.0'),
        ('5.0', '-6.0', '0.0'),
        ('5.0', '6.0', '0.0'),
        ('-5.0', '6.5', '1.5708'),
        ('5.0', '-6.5', '1.5708'),
    ],
    'corridor': [
        ('0.5', '0.0', '0.0'),
        ('0.0', '-2.5', '1.5708'),
        ('3.45', '2.5', '1.5708'),
        ('-3.45', '2.5', '1.5708'),
        ('-5.55', '2.5', '1.5708'),
        ('5.55', '2.5', '1.5708'),
        ('-4.5', '-2.5', '1.5708'),
        ('4.5', '-2.5', '1.5708'),
        ('0.0', '-1.0', '0.0'),
        ('0.0', '1.0', '0.0'),
    ],
    'bookstore': [
        ('10.0', '-9.3', '0.0'),
        ('10.0', '-6.3', '1.5708'),
        ('0.0', '9.3', '0.0'),
        ('-6.0', '6.3', '1.5708'),
        ('-2.0', '6.3', '1.5708'),
        ('2.0', '6.3', '1.5708'),
        ('6.0', '6.3', '1.5708'),
        ('-6.0', '-6.3', '1.5708'),
        ('-2.0', '-6.3', '1.5708'),
        ('2.0', '-6.3', '1.5708'),
    ],
    'warehouse': [
        ('0.0', '-0.75', '0.0'),
        ('-2.8', '0.0', '1.5708'),
        ('2.8', '0.0', '1.5708'),
        ('0.0', '4.2', '0.0'),
        ('0.0', '5.5', '0.0'),
        ('0.0', '-4.0', '0.0'),
        ('0.0', '-5.5', '0.0'),
        ('-8.2', '0.0', '1.5708'),
        ('8.2', '0.0', '1.5708'),
        ('0.0', '1.3', '0.0'),
    ],
    'warehouse_detailed': [
        ('0.0', '0.0', '0.0'),
        ('2.5', '0.0', '1.5708'),
        ('0.0', '2.5', '0.0'),
        ('2.5', '3.5', '1.5708'),
        ('0.0', '-4.0', '0.0'),
        ('2.5', '-3.0', '1.5708'),
        ('-6.0', '8.5', '0.0'),
        ('-6.0', '5.5', '1.5708'),
        ('-6.0', '-9.2', '0.0'),
        ('-6.0', '-6.2', '1.5708'),
    ],
}


def _launch(context):
    world_name = LaunchConfiguration('world').perform(context)
    if world_name not in WORLD_FILES:
        raise RuntimeError('world must be one of: ' + ', '.join(WORLD_FILES))
    gui_render_engine = LaunchConfiguration('gui_render_engine').perform(context)
    if gui_render_engine not in ('ogre', 'ogre2'):
        raise RuntimeError('gui_render_engine must be either "ogre" or "ogre2"')
    assets = get_package_share_directory('amr_simulation_assets')
    mir_description = get_package_share_directory('mir_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    directory, filename = WORLD_FILES[world_name]
    world_file = os.path.join(assets, 'worlds', directory, filename)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    dynamic_obstacle = LaunchConfiguration('dynamic_obstacle').perform(context).lower() == 'true'
    worker_count = FIXED_OBSTACLE_COUNTS.get(world_name, 1 if dynamic_obstacle else 0)
    base_world_name = WORLD_BASE_NAMES[world_name]
    gz_world_name = BASE_GZ_WORLD_NAMES[base_world_name]
    worker_spawns = WORKER_SPAWNS_BY_WORLD[base_world_name]
    x = LaunchConfiguration('x').perform(context) or SPAWN_POSES[world_name][0]
    y = LaunchConfiguration('y').perform(context) or SPAWN_POSES[world_name][1]
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(mir_description, 'urdf', 'mir.urdf.xacro')]), value_type=str)
    resource_path = ':'.join(filter(None, [
        assets,
        os.path.join(assets, 'models'),
        os.path.dirname(mir_description),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))
    # Isolate every launch from stale Gazebo servers. Without a unique
    # transport partition, a new GUI can attach to an older world that was
    # left running after an interrupted launch.
    partition = f'amr_simulation_{os.getpid()}'
    gz_args = ('-r -s ' if headless else f'-r --render-engine-gui {gui_render_engine} ') + world_file
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('GZ_PARTITION', partition),
        LogInfo(msg=f'Launching world "{world_name}" from: {world_file}'),
        LogInfo(msg=f'Gazebo transport partition: {partition}'),
        LogInfo(msg=f'Dynamic obstacle count: {worker_count}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args,
                              'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': robot_description}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge', output='screen',
             arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                 '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                 '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                 '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                 '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                 '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/gazebo_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                 '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                 '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                 f'/world/{gz_world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
             remappings=[('/gazebo_tf', '/tf'), ('/scan', LaunchConfiguration('scan_topic'))]),
        TimerAction(period=2.0, actions=[Node(package='ros_gz_sim', executable='create', name='spawn_mir', output='screen',
             arguments=['-name', 'mir', '-topic', '/robot_description', '-x', x, '-y', y, '-z', '0.20'])]),
    ]
    worker_sdf = os.path.join(assets, 'actors', 'warehouse_worker', 'worker.sdf')
    for index, (worker_x, worker_y, worker_yaw) in enumerate(worker_spawns[:worker_count], start=1):
        actions.append(TimerAction(period=4.0 + 0.35 * (index - 1), actions=[Node(
            package='ros_gz_sim', executable='create', name=f'spawn_warehouse_worker_{index}', output='screen',
            arguments=['-file', worker_sdf, '-name', f'warehouse_worker_{index}',
                       '-x', worker_x, '-y', worker_y, '-z', '0', '-Y', worker_yaw])]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
            description='Base world name, or any base name suffixed with _0, _3, or _10'),
        DeclareLaunchArgument('dynamic_obstacle', default_value='false',
            description='Legacy switch: spawn one worker for world names without a fixed _0/_3/_10 count'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('gui_render_engine', default_value='ogre',
            description='Gazebo GUI renderer: ogre is the VM-compatible default; ogre2 is optional'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
            description='ROS output topic for the Gazebo LiDAR bridge'),
        DeclareLaunchArgument('x', default_value='', description='Override world-specific robot x spawn pose'),
        DeclareLaunchArgument('y', default_value='', description='Override world-specific robot y spawn pose'),
        OpaqueFunction(function=_launch),
    ])
