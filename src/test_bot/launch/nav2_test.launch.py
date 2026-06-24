"""
nav2_test.launch.py - prueba de nav2 SIN el ESP32 (usa fake_odom).

Levanta:
  - robot_state_publisher (URDF real, para TF base_link->sensores)
  - fake_odom            (/cmd_vel -> pose integrada -> /odometry/filtered_odom
                          + /odom + TF odom->base_link + TF estatico map->odom)
  - map_server           (carga config/test_map_<map_name>.yaml)
  - nav2: planner + controller + recoveries + bt_navigator + waypoint_follower
  - lifecycle managers   (activan todo automaticamente)

Asi probas el lazo completo de nav2 en software: mandas un goal y el "robot"
(fake_odom) navega hacia el. Requiere la imagen con BUILD_NAV2=1.

Uso:
  ros2 launch test_bot nav2_test.launch.py
  ros2 launch test_bot nav2_test.launch.py map_name:=large
  # mandar un goal:
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}'
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

PKG = get_package_share_directory('test_bot')


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')

    nav2_params = os.path.join(PKG, 'config', 'nav2_params.yaml')
    map_yaml = [PKG, '/config/test_map_', map_name, '.yaml']
    # OJO: la ruta del BT XML (default_bt_xml_filename) va FIJA en nav2_params.yaml
    # apuntando a /opt/nav2_ws/...  (override por launch no funcionaba en Foxy).

    xacro_file = os.path.join(PKG, 'description', 'robot_real.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    fake_odom = Node(
        package='test_bot', executable='fake_odom', name='fake_odom',
        output='screen',
        parameters=[{
            'odom_topic': '/odometry/filtered_odom',
            'odom_frame': 'odom', 'base_frame': 'base_link', 'map_frame': 'map',
            'rate': 30.0, 'publish_tf': True, 'publish_map_odom': True,
        }],
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False, 'yaml_filename': map_yaml}],
    )
    planner = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=[nav2_params],
    )
    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', parameters=[nav2_params],
    )
    recoveries = Node(
        package='nav2_recoveries', executable='recoveries_server',
        name='recoveries_server', output='screen', parameters=[nav2_params],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'default_bt_xml_filename': bt_xml}],
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=[nav2_params],
    )

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'node_names': ['map_server']}],
    )
    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'node_names': ['planner_server', 'controller_server',
                                    'recoveries_server', 'bt_navigator',
                                    'waypoint_follower']}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='small'),
        rsp, fake_odom, map_server,
        planner, controller, recoveries, bt_navigator, waypoint_follower,
        lifecycle_localization, lifecycle_navigation,
    ])
