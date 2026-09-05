from pathlib import Path
import py_compile

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_launch_and_scripts_compile():
    py_compile.compile(str(PACKAGE / 'launch' / 'nav2_world.launch.py'), doraise=True)
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
