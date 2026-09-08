from pathlib import Path
import importlib.util
import math
import py_compile

from geometry_msgs.msg import Pose
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def _load_ground_truth_localizer():
    path = PACKAGE / 'scripts' / 'gazebo_ground_truth_localizer.py'
    spec = importlib.util.spec_from_file_location('gazebo_ground_truth_localizer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_and_scripts_compile():
    py_compile.compile(str(PACKAGE / 'launch' / 'nav2_world.launch.py'), doraise=True)
    py_compile.compile(
        str(PACKAGE / 'scripts' / 'gazebo_ground_truth_localizer.py'), doraise=True)
    py_compile.compile(str(PACKAGE / 'scripts' / 'initial_pose_publisher.py'), doraise=True)
    py_compile.compile(str(PACKAGE / 'scripts' / 'generate_maps.py'), doraise=True)


def test_nav2_velocity_output_is_safety_gated():
    parameters = yaml.safe_load((PACKAGE / 'config' / 'nav2_params.yaml').read_text())
    collision_monitor = parameters['collision_monitor']['ros__parameters']
    assert collision_monitor['cmd_vel_out_topic'] == '/navigation/cmd_vel'
    launch_source = (PACKAGE / 'launch' / 'nav2_world.launch.py').read_text()
    assert "'input_cmd_vel_topic': '/navigation/cmd_vel'" in launch_source
    assert "'output_cmd_vel_topic': '/safety/cmd_vel'" in launch_source
    assert "'cmd_vel_topic': '/safety/cmd_vel'" in launch_source


def test_simulation_launch_recovers_after_dynamic_obstacle_clears():
    launch_source = (
        PACKAGE / 'launch' / 'nav2_world.launch.py').read_text(encoding='utf-8')
    parameters = yaml.safe_load(
        (PACKAGE / 'config' / 'nav2_params.yaml').read_text(encoding='utf-8'))

    for name in ('local_costmap', 'global_costmap'):
        scan = parameters[name][name]['ros__parameters']['obstacle_layer']['scan']
        assert scan['clearing'] is True
        assert scan['inf_is_valid'] is True

    assert "'auto_resume_on_safety_recovery': auto_resume" in launch_source
    assert "'max_navigation_retries': navigation_retries" in launch_source
    assert "DeclareLaunchArgument(\n            'auto_resume', default_value='true'" in launch_source


def test_validator_observes_nav2_controller_tracking_path():
    launch_source = (PACKAGE / 'launch' / 'nav2_world.launch.py').read_text()
    assert "'local_path_topic': '/received_global_plan'" in launch_source


def test_nav2_frames_and_sensor_topics_match_simulation():
    parameters = yaml.safe_load((PACKAGE / 'config' / 'nav2_params.yaml').read_text())
    amcl = parameters['amcl']['ros__parameters']
    assert amcl['global_frame_id'] == 'map'
    assert amcl['odom_frame_id'] == 'odom'
    assert amcl['base_frame_id'] == 'base_footprint'
    assert amcl['scan_topic'] == '/scan'
    for costmap_name in ('local_costmap', 'global_costmap'):
        costmap = parameters[costmap_name][costmap_name]['ros__parameters']
        assert costmap['robot_base_frame'] == 'base_footprint'
        assert costmap['obstacle_layer']['scan']['topic'] == '/scan'


def test_development_nav2_msgs_cannot_shadow_system_package():
    assert (PACKAGE.parent / 'nav2_msgs' / 'COLCON_IGNORE').exists()


def test_nav2_python_expression_booleans_are_capitalized():
    launch_source = (PACKAGE / 'launch' / 'nav2_world.launch.py').read_text()
    assert "'slam': 'False'" in launch_source
    assert "'use_localization': 'True'" in launch_source
    assert "'use_composition': 'False'" in launch_source
    assert "'use_respawn': 'False'" in launch_source


def test_simulation_localization_cannot_jump_map_to_odom():
    launch_source = (
        PACKAGE / 'launch' / 'nav2_world.launch.py').read_text(encoding='utf-8')

    assert "'localization_mode', default_value='simulation'" in launch_source
    assert "executable='gazebo_ground_truth_localizer.py'" in launch_source
    assert "'ground_truth_topic': '/simulation/dynamic_pose'" in launch_source
    assert "'node_names': ['map_server']" in launch_source
    assert "os.path.join(nav2_share, 'launch', 'navigation_launch.py')" in launch_source
    assert "os.path.join(nav2_share, 'launch', 'bringup_launch.py')" in launch_source


def test_simulation_bridges_physical_pose_and_workers_yield_to_robot():
    simulation_source = (
        PACKAGE.parent / 'amr_simulation' / 'launch' /
        'simulation.launch.py').read_text(encoding='utf-8')
    worker_source = (
        PACKAGE.parent / 'amr_simulation' / 'scripts' /
        'worker_motion_controller.py').read_text(encoding='utf-8')

    assert "dynamic_pose/info@tf2_msgs/msg/TFMessage" in simulation_source
    assert "'/simulation/dynamic_pose'" in simulation_source
    assert "'robot_clearance': 0.90" in simulation_source
    assert '_would_enter_robot_clearance' in worker_source


def test_ground_truth_correction_maps_odom_pose_onto_physical_pose():
    module = _load_ground_truth_localizer()
    odom = Pose()
    odom.position.x = 2.0
    ground_truth = Pose()
    ground_truth.position.x = 1.0
    ground_truth.position.y = 3.0
    ground_truth.orientation.z = math.sin(math.pi / 4.0)
    ground_truth.orientation.w = math.cos(math.pi / 4.0)

    x, y, yaw = module.compute_map_to_odom(ground_truth, odom)

    assert math.isclose(x, 1.0, abs_tol=1.0e-9)
    assert math.isclose(y, 1.0, abs_tol=1.0e-9)
    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1.0e-9)
