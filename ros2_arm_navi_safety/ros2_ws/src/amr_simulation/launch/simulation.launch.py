"""Launch one self-contained AMR world and a MiR100 via Gazebo Harmonic.

No Nav2, AMCL, SLAM, mission, or safety node is included here by design.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


# Keep generated world / URDF directories alive until the launch process exits.
_GENERATED_DIRECTORIES = []


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

# Each tuple is the centre and heading of a six-metre collision-aware worker
# path. The worker model starts three metres behind the centre and its
# TrajectoryFollower moves along local +X. The first three entries are also used
# by each *_3 scenario.
WORKER_SPAWNS_BY_WORLD = {
    'empty': [
        ('0.0', '0.0', '3.14159265'),
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
        ('0.0', '-0.75', '3.14159265'),
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
    robot_xacro = os.path.join(mir_description, 'urdf', 'mir.urdf.xacro')
    robot_description_xml = xacro.process_file(robot_xacro).toxml()
    robot_description = ParameterValue(robot_description_xml, value_type=str)
    worker_sdf = os.path.join(assets, 'actors', 'warehouse_worker', 'worker.sdf')

    # Rendering sensors and moving models inserted after Gazebo starts can be
    # absent from the Ogre2 sensor scene. Build one private, expanded world so
    # every entity is present before the renderer initializes.
    generated_directory = tempfile.TemporaryDirectory(prefix='amr_simulation_')
    _GENERATED_DIRECTORIES.append(generated_directory)
    generated_path = Path(generated_directory.name)
    robot_urdf = generated_path / 'mir.urdf'
    robot_urdf.write_text(robot_description_xml, encoding='utf-8')
    robot_include = (
        '\n    <include>\n'
        f'      <uri>{robot_urdf.as_uri()}</uri>\n'
        '      <name>mir</name>\n'
        f'      <pose>{x} {y} 0.20 0 0 0</pose>\n'
        '    </include>\n'
        '    <model name="mir_lidar_proxy">\n'
        '      <static>true</static>\n'
        f'      <pose>{x} {y} 0 0 0 0</pose>\n'
        '      <link name="link">\n'
        '        <sensor name="front_laser_sensor" type="gpu_lidar">\n'
        '          <pose>0.509646 0 0.352 0 0 0</pose>\n'
        '          <topic>/scan</topic><gz_frame_id>front_laser_link</gz_frame_id>\n'
        '          <update_rate>30</update_rate><always_on>true</always_on>\n'
        '          <lidar><scan>\n'
        '            <horizontal><samples>720</samples><resolution>1</resolution>'
        '<min_angle>-2.35619449</min_angle><max_angle>2.35619449</max_angle></horizontal>\n'
        '            <vertical><samples>2</samples><resolution>1</resolution>'
        '<min_angle>-0.001</min_angle><max_angle>0.001</max_angle></vertical>\n'
        '          </scan><range><min>0.1</min><max>30</max><resolution>0.01</resolution></range>'
        '<noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise></lidar>\n'
        '        </sensor>\n'
        '      </link>\n'
        '    </model>\n'
    )
    worker_includes = []
    worker_paths = []
    for index, (worker_x, worker_y, worker_yaw) in enumerate(
            worker_spawns[:worker_count], start=1):
        centre_x = float(worker_x)
        centre_y = float(worker_y)
        yaw = float(worker_yaw)
        start_x = centre_x - 3.0 * math.cos(yaw)
        start_y = centre_y - 3.0 * math.sin(yaw)
        end_x = centre_x + 3.0 * math.cos(yaw)
        end_y = centre_y + 3.0 * math.sin(yaw)
        worker_name = f'warehouse_worker_{index}'
        worker_paths.append((worker_name, start_x, start_y, end_x, end_y))
        worker_includes.append(
            '    <include>\n'
            f'      <uri>{Path(worker_sdf).as_uri()}</uri>\n'
            f'      <name>{worker_name}</name>\n'
            f'      <pose>{start_x} {start_y} 0.02 0 0 {worker_yaw}</pose>\n'
            '    </include>\n')
    robot_include += ''.join(worker_includes)
    included_world = generated_path / f'included_{filename}'
    world_xml = Path(world_file).read_text(encoding='utf-8')
    if '</world>' not in world_xml:
        raise RuntimeError(f'World file has no closing </world> tag: {world_file}')
    included_world.write_text(
        world_xml.replace('</world>', robot_include + '  </world>', 1),
        encoding='utf-8')
    expanded = subprocess.run(
        ['gz', 'sdf', '-p', str(included_world)],
        check=True, capture_output=True, text=True)
    expanded_root = ElementTree.fromstring(expanded.stdout)
    mir_model = expanded_root.find("./world/model[@name='mir']")
    if mir_model is None:
        raise RuntimeError('Generated world does not contain the MiR model')
    for link in mir_model.findall('.//link'):
        for sensor in list(link.findall('sensor')):
            if sensor.get('type') in ('gpu_lidar', 'camera', 'depth_camera', 'rgbd_camera'):
                link.remove(sensor)
    for visual in mir_model.findall('.//visual'):
        flags = visual.find('visibility_flags')
        if flags is None:
            flags = ElementTree.SubElement(visual, 'visibility_flags')
        flags.text = '0x02'
    for lidar in expanded_root.findall(".//sensor[@type='gpu_lidar']/lidar"):
        mask = lidar.find('visibility_mask')
        if mask is None:
            mask = ElementTree.SubElement(lidar, 'visibility_mask')
        mask.text = '0x01'
    generated_world = generated_path / filename
    ElementTree.ElementTree(expanded_root).write(
        generated_world, encoding='utf-8', xml_declaration=True)
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
    # GPU LiDAR is a rendering sensor. Server-only mode needs explicit headless
    # rendering; `-s` alone can leave every scan ray at +inf.
    gz_args = (
        '-r -s --headless-rendering '
        if headless else f'-r --render-engine-gui {gui_render_engine} '
    ) + str(generated_world)
    worker_path_parameters = json.dumps([
        {
            'name': name,
            'start_x': start_x,
            'start_y': start_y,
            'end_x': end_x,
            'end_y': end_y,
        }
        for name, start_x, start_y, end_x, end_y in worker_paths
    ])
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
                 f'/world/{gz_world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                 '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                 '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                 f'/world/{gz_world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
             remappings=[
                 ('/gazebo_tf', '/tf'),
                 (f'/world/{gz_world_name}/dynamic_pose/info',
                  '/simulation/dynamic_pose'),
                 ('/scan', '/scan/rendered'),
                 ('/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
             ]),
        Node(package='amr_simulation', executable='worker_motion_controller',
             name='worker_motion_controller', output='screen', parameters=[{
                 'set_pose_service': f'/world/{gz_world_name}/set_pose',
                 'paths_json': worker_path_parameters,
                 'speed': 0.35,
                 'odom_topic': '/odom',
                 'ground_truth_topic': '/simulation/dynamic_pose',
                 'robot_clearance': 0.90,
             }]),
        Node(package='amr_simulation', executable='scan_2d_projector', name='scan_2d_projector',
             output='screen', remappings=[('/scan', LaunchConfiguration('scan_topic'))],
             parameters=[{
                 'render_dropout_hold_sec': 0.25,
                 'set_pose_service': f'/world/{gz_world_name}/set_pose',
                 'world_start_x': float(x),
                 'world_start_y': float(y),
                 'worker_count': worker_count,
                 'worker_radius': 0.30,
             }]),
    ]
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
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
            description='ROS input topic for the Gazebo velocity bridge'),
        DeclareLaunchArgument('x', default_value='', description='Override world-specific robot x spawn pose'),
        DeclareLaunchArgument('y', default_value='', description='Override world-specific robot y spawn pose'),
        OpaqueFunction(function=_launch),
    ])
