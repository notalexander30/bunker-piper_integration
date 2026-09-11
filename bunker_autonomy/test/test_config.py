from pathlib import Path
import ast

import yaml


def test_autonomy_yaml_loads_expected_parameters():
    config_path = Path(__file__).parents[1] / 'config' / 'autonomy.yaml'
    data = yaml.safe_load(config_path.read_text())

    assert data['safety_monitor_node']['ros__parameters']['pointcloud_topic'] == (
        '/cx/lslidar_point_cloud'
    )
    assert data['safety_monitor_node']['ros__parameters']['base_frame'] == 'laser_link'
    assert data['safety_monitor_node']['ros__parameters']['hard_stop_box'] == {
        'x_min': 0.2,
        'x_max': 0.4,
        'y_min': -0.5,
        'y_max': 0.5,
        'use_z_filter': False,
    }
    assert data['safety_monitor_node']['ros__parameters']['hard_stop_point_threshold'] == 30
    assert data['safety_monitor_node']['ros__parameters']['route_block_point_threshold'] == 40
    assert data['safety_monitor_node']['ros__parameters'][
        'route_block_point_thresholds'
    ] == {
        'front': 240,
        'back': 80,
        'left': 120,
        'right': 120,
    }
    assert data['safety_monitor_node']['ros__parameters']['route_box']['z_min'] == 0.15
    assert data['safety_monitor_node']['ros__parameters']['route_box']['front'] == {
        'x_min': 0.2,
        'x_max': 0.4,
        'y_min': -0.5,
        'y_max': 0.5,
    }
    assert data['safety_monitor_node']['ros__parameters']['route_box']['back'] == {
        'x_min': -0.4,
        'x_max': -0.2,
        'y_min': -0.5,
        'y_max': 0.5,
    }
    assert data['safety_monitor_node']['ros__parameters']['route_box']['left'] == {
        'x_min': -0.5,
        'x_max': 0.5,
        'y_min': 0.2,
        'y_max': 0.4,
    }
    assert data['safety_monitor_node']['ros__parameters']['route_box']['right'] == {
        'x_min': -0.5,
        'x_max': 0.5,
        'y_min': -0.4,
        'y_max': -0.2,
    }
    assert data['safety_monitor_node']['ros__parameters']['safety_stop_topic'] == (
        '/lidar_safety_stop'
    )
    assert data['safety_monitor_node']['ros__parameters']['route_status_topic'] == (
        '/lidar_route_status'
    )
    assert data['depth_route_monitor_node']['ros__parameters']['depth_image_topic'] == (
        '/camera/camera/depth/image_rect_raw'
    )
    depth = data['depth_route_monitor_node']['ros__parameters']
    assert depth['min_depth_m'] == 0.075
    assert depth['max_depth_m'] == 3.0
    assert depth['hard_stop_distance_m'] == 0.6
    assert depth['route_block_distance_m'] == 0.6
    assert data['depth_route_monitor_node']['ros__parameters']['input_timeout_sec'] == 1.0
    assert data['sensor_fusion_node']['ros__parameters']['safety_stop_topic'] == '/safety_stop'
    assert data['sensor_fusion_node']['ros__parameters']['route_status_topic'] == '/route_status'
    assert data['sensor_fusion_node']['ros__parameters']['safety_stop_reason_topic'] == (
        '/safety_stop_reason'
    )
    assert data['vlm_goal_monitor_node']['ros__parameters']['vlm_result_topic'] == '/vlm_result'
    assert data['vlm_goal_monitor_node']['ros__parameters']['target_keywords'] == [
        'trash can visible'
    ]
    assert data['search_behavior_node']['ros__parameters']['planner_mode'] == 'vla'
    assert data['search_behavior_node']['ros__parameters']['route_status_topic'] == (
        '/route_status'
    )
    assert data['search_behavior_node']['ros__parameters']['vla_max_linear_speed_mps'] == 0.06
    assert data['search_behavior_node']['ros__parameters']['vla_max_angular_speed_radps'] == 0.15
    assert data['search_behavior_node']['ros__parameters']['approach_stop_distance_m'] == 0.30
    assert data['cmd_vel_mux_node']['ros__parameters']['command_timeout_sec'] == 0.5
    assert data['safety_monitor_node']['ros__parameters']['corridor']['width_m'] == 0.60
    assert data['cmd_vel_mux_node']['ros__parameters']['allow_reverse'] is False
    assert data['cmd_vel_mux_node']['ros__parameters']['max_linear_speed_mps'] == 0.06
    assert data['cmd_vel_mux_node']['ros__parameters']['max_angular_speed_radps'] == 0.15
    assert data['cmd_vel_mux_node']['ros__parameters']['output_cmd_vel_topic'] == '/cmd_vel'
    assert data['cmd_vel_mux_node']['ros__parameters']['linear_acceleration_mps2'] == 0.06
    assert data['cmd_vel_mux_node']['ros__parameters']['linear_deceleration_mps2'] == 0.125
    assert data['operator_status_node']['ros__parameters']['status_topic'] == (
        '/autonomy_notice'
    )
    assert data['data_logger_node']['ros__parameters']['operator_label_topic'] == (
        '/operator_label'
    )
    assert data['data_logger_node']['ros__parameters']['safety_event_prefix'] == (
        'safety_events'
    )
    assert data['data_logger_node']['ros__parameters']['perception_trace_prefix'] == (
        'perception_trace'
    )


def test_vla_prompt_requests_dynamic_sensor_informed_bounded_actions():
    config_path = Path(__file__).parents[1] / 'config' / 'simple_vlm_vla.yaml'
    config = yaml.safe_load(config_path.read_text())
    prompt = config['prompt']

    assert config['api_key'] is None
    assert 'vision-language-action planner' in prompt
    assert 'SENSOR_CONTEXT' in prompt
    assert 'linear_velocity_mps' in prompt
    assert 'angular_velocity_radps' in prompt
    assert 'target_bbox_norm' in prompt
    assert 'do not follow a fixed' in prompt
    assert 'never' in prompt.lower() and 'revers' in prompt.lower()


def test_d435i_camera_only_profile_uses_aligned_depth_and_no_lidar_config():
    config_dir = Path(__file__).parents[1] / 'config'
    camera = yaml.safe_load((config_dir / 'realsense_d435i.yaml').read_text())
    autonomy = yaml.safe_load((config_dir / 'autonomy_d435i.yaml').read_text())

    assert camera['device_type'] == 'd435i'
    assert camera['enable_color'] is True
    assert camera['enable_depth'] is True
    assert camera['enable_gyro'] is False
    assert camera['enable_accel'] is False
    assert camera['align_depth.enable'] is True
    assert camera['pointcloud.enable'] is False

    assert 'safety_monitor_node' not in autonomy
    depth = autonomy['depth_route_monitor_node']['ros__parameters']
    assert depth['depth_image_topic'] == (
        '/front_camera/aligned_depth_to_color/image_raw'
    )
    assert depth['front_depth_image_topic'] == (
        '/front_camera/aligned_depth_to_color/image_raw'
    )
    assert depth['rear_depth_image_topic'] == (
        '/rear_camera/aligned_depth_to_color/image_raw'
    )
    assert depth['active_camera_topic'] == '/landmark_navigator/active_camera'
    assert depth['active_depth_camera'] == 'front'
    assert depth['camera_view_name'] == 'arm_top_forward'
    assert depth['hard_stop_distance_m'] == 0.05
    assert depth['route_block_distance_m'] == 0.05
    assert depth['allow_front_obstacle_reroute'] is False
    assert depth['roi_top_fraction'] < depth['roi_bottom_fraction']
    assert depth['left_sector_end_fraction'] < depth['right_sector_start_fraction']
    assert autonomy['sensor_fusion_node']['ros__parameters']['fusion_mode'] == (
        'depth_only'
    )
    assert autonomy['search_behavior_node']['ros__parameters'][
        'target_timeout_sec'
    ] == 8.0


def test_door_profile_uses_forward_exploration_and_stops_when_door_is_confirmed():
    config_path = Path(__file__).parents[1] / 'config' / 'door_search_d435i.yaml'
    config = yaml.safe_load(config_path.read_text())
    behavior = config['search_behavior_node']['ros__parameters']

    assert 'vlm_goal_monitor_node' not in config
    assert behavior['planner_mode'] == 'forward_exploration'
    assert behavior['complete_on_target_detection'] is False
    assert behavior['approach_stop_distance_m'] == 1.00
    assert behavior['required_close_confirmations'] == 2
    assert behavior['front_block_confirmation_sec'] == 0.75
    depth = config['depth_route_monitor_node']['ros__parameters']
    assert depth['max_depth_m'] == 6.0
    assert depth['minimum_valid_fraction'] == 0.10
    assert depth['hard_stop_distance_m'] == 0.05
    assert depth['route_block_distance_m'] == 0.05
    assert depth['allow_front_obstacle_reroute'] is False
    assert behavior['search_linear_speed'] == 0.04
    assert config['cmd_vel_mux_node']['ros__parameters']['max_linear_speed_mps'] == 0.04


def test_door_launch_has_a_dedicated_yolo_rviz_node():
    launch_path = Path(__file__).parents[1] / 'launch' / 'find_door_d435i.launch.py'
    source = launch_path.read_text()

    ast.parse(source)
    assert "name='door_yolo_rviz'" in source
    assert "'camera_rviz': 'false'" in source
    assert "'door_yolo_d435i.launch.py'" in source
    assert "'weights_path': LaunchConfiguration('weights_path')" not in source
    assert "'port_name': LaunchConfiguration('bunker_can')" in source
