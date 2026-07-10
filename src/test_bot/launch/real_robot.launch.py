"""
real_robot.launch.py - bringup del robot REAL en la Jetson (ROS2 Foxy, Docker).

  - robot_state_publisher (robot_real.urdf.xacro, sin lidar/gazebo)
  - csi_camera_node       (camara CSI -> /camera/image_raw + video H264 a la GUI)
  - aruco_localizer       (-> /aruco_pose)
  - esp32_serial_bridge   (serie ESP32 -> /odom [calculado en Jetson] ; /cmd_vel -> WHEEL_VEL_CMD por rueda)
  - ekf x2 (robot_localization): local (odom->base_link) y global (map->odom)
  - nav2 (map_server+planner+controller+recoveries+bt_navigator+waypoint_follower)
  - teleop_gateway + gui_bridge_node (puentes hacia capbot-host)

Flag enable_motion (default true):
  - true  : Corren esp32_bridge + EKF; el TF lo dan los EKF (aruco con
            publish_tf:=false). Requerido para enable_nav (nav2 necesita
            /odometry/filtered_odom y el TF map->odom->base_link de los EKF).
  - false : Camara-only (sin hardware). aruco publica TF map->base_link
            directo; no se lanzan esp32_bridge, EKF ni nav2.

Flag enable_nav (default true, requiere enable_motion:=true):
  - true  : nav2 completo + map_server (config/test_map_<map_name>.yaml).
            gui_bridge_node ya corre siempre con enable_motion; con nav2
            arriba, navigate_to_pose queda disponible para los goals de la GUI.
  - false : sin nav2 (ahorra CPU del Jetson durante teleop puro).

Uso:
  ros2 launch test_bot real_robot.launch.py
  ros2 launch test_bot real_robot.launch.py enable_nav:=false   # sin nav2
  ros2 launch test_bot real_robot.launch.py enable_motion:=false   # solo camara/aruco
  ros2 launch test_bot real_robot.launch.py map_name:=large serial_port:=/dev/ttyUSB0
  HOST_IP=192.168.1.10 ros2 launch test_bot real_robot.launch.py   # video a la GUI
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro

PKG = get_package_share_directory('test_bot')


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')
    markers_db = LaunchConfiguration('markers_db')
    enable_motion = LaunchConfiguration('enable_motion')
    enable_nav = LaunchConfiguration('enable_nav')
    serial_port = LaunchConfiguration('serial_port')

    camera_info = 'file://' + os.path.join(PKG, 'config', 'camera.yaml')
    ekf_params = os.path.join(PKG, 'config', 'ekf_real.yaml')
    nav2_params = os.path.join(PKG, 'config', 'nav2_params.yaml')
    map_yaml = [PKG, '/config/test_map_', map_name, '.yaml']

    xacro_file = os.path.join(PKG, 'description', 'robot_real.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    camera = Node(
        package='test_bot', executable='csi_camera_node',
        name='csi_camera_node', output='screen',
        parameters=[{
            'sensor_id': 0,
            'capture_width': 1280, 'capture_height': 720,
            'output_width': 640, 'output_height': 480,
            'framerate': 30, 'flip_method': 0,
            'frame_id': 'camera_link_optical',
            'camera_info_url': camera_info,
            'enable_video': True,
            'host_ip': '',          # vacio => usa la env HOST_IP
            'video_port': 5000,
            'video_bitrate_kbps': 4000,
        }],
    )

    delayed_camera = TimerAction(
        period=15.0,
        actions=[camera]
    )

    # Parametros comunes de aruco (publish_tf se setea segun enable_motion).
    aruco_params = {
        'use_sim_time': False,
        'markers_db': markers_db,
        'image_topic': '/camera/image_raw',
        'camera_info_topic': '/camera/camera_info',
        'camera_frame': 'camera_link_optical',
        'base_frame': 'base_link',
        'odom_frame': 'odom',
        'map_frame': 'map',
        'max_distance': 0.45,
        # 27 es muy laxo a proposito: con camera.yaml SIN calibrar el error de
        # reproyeccion es alto. RECALIBRAR camera.yaml y bajar a ~3.0.
        'max_reproj_error_px': 60.0,
        # Filtro de distancia REAL independiente de calibracion: el area en px
        # no depende de los intrinsecos. Con marcador de 10 cm y f~500,
        # area ~ (500*0.1/d)^2  ->  0.45 m ~ 12000 px2, 0.7 m ~ 5000 px2.
        # Tunear mirando "area=" en el log OK del aruco_localizer.
        'min_marker_area_px': 8000.0,
        'filter_window': 1,
        'ambiguity_ratio_threshold': 1.5,
    }
    # Fase 1 (enable_motion:=false): ArUco publica el TF map->base_link.
    aruco_tf = Node(
        package='test_bot', executable='aruco_localizer',
        name='aruco_localizer', output='screen',
        parameters=[dict(aruco_params, publish_tf=True)],
        condition=UnlessCondition(enable_motion),
    )
    # Fase 2 (enable_motion:=true): el EKF da el TF; ArUco solo /aruco_pose.
    aruco_notf = Node(
        package='test_bot', executable='aruco_localizer',
        name='aruco_localizer', output='screen',
        parameters=[dict(aruco_params, publish_tf=False)],
        condition=IfCondition(enable_motion),
    )

    esp32 = Node(
        package='test_bot', executable='esp32_serial_bridge',
        name='esp32_serial_bridge', output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baudrate': 115200,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_odom_tf': False,   # el EKF local da odom->base_link
            'max_linear_speed': 0.6,    # m/s; ajustar a la velocidad real
            'max_angular_speed': 3.0,   # rad/s; idem
            'cmd_vel_timeout': 0.5,
            # Deben calzar con description/robot_core.xacro (radio/separacion
            # de ruedas) y capbot-ESP32/include/Config.h (Cfg::WHEEL_CPR).
            'wheel_radius': 0.035,      # m
            'wheel_separation': 0.17,   # m
            'wheel_cpr': 910,           # cuentas/vuelta (cuadratura 4x)
        }],
        condition=IfCondition(enable_motion),
    )

    ekf_odom = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node_odom', output='screen',
        parameters=[ekf_params],
        remappings=[('/odometry/filtered', '/odometry/filtered_odom')],
        condition=IfCondition(enable_motion),
    )
    ekf_map = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node_map', output='screen',
        parameters=[ekf_params],
        remappings=[('/odometry/filtered', '/odometry/filtered_map')],
        condition=IfCondition(enable_motion),
    )

    # Gateway hacia la GUI (capbot-host), reexpone los protocolos de teleop:
    #   - teleop_gateway: UDP joystick/e-stop/PID (5005/6) + WS telemetria (8765)
    #   - gui_bridge_node: WS nav (8766) -> pose (TF) siempre + goals (cuando haya nav2)
    teleop_gw = Node(
        package='test_bot', executable='teleop_gateway', name='teleop_gateway',
        output='screen', condition=IfCondition(enable_motion),
    )
    gui_bridge = Node(
        package='test_bot', executable='gui_bridge_node', name='gui_bridge_node',
        output='screen',
        parameters=[{'ws_port': 8766, 'map_frame': 'map',
                     'base_frame': 'base_link', 'odom_frame': 'odom',
                     'map_name': map_name}],
        condition=IfCondition(enable_motion),
    )

    # ---- nav2 (Fase 3): mapa estatico, sin lidar (solo static_layer+inflation). ----
    # nav2 requiere el TF map->odom->base_link y /odometry/filtered_odom de los
    # EKF, asi que solo corre si enable_motion Y enable_nav son ambos true.
    nav_enabled = PythonExpression(
        ['"', enable_motion, '" == "true" and "', enable_nav, '" == "true"'])

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False, 'yaml_filename': map_yaml}],
        condition=IfCondition(nav_enabled),
    )
    planner = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=[nav2_params],
        condition=IfCondition(nav_enabled),
    )
    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', parameters=[nav2_params],
        condition=IfCondition(nav_enabled),
    )
    recoveries = Node(
        package='nav2_recoveries', executable='recoveries_server',
        name='recoveries_server', output='screen', parameters=[nav2_params],
        condition=IfCondition(nav_enabled),
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', parameters=[nav2_params],
        condition=IfCondition(nav_enabled),
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=[nav2_params],
        condition=IfCondition(nav_enabled),
    )
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'node_names': ['map_server']}],
        condition=IfCondition(nav_enabled),
    )
    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'node_names': ['planner_server', 'controller_server',
                                    'recoveries_server', 'bt_navigator',
                                    'waypoint_follower']}],
        condition=IfCondition(nav_enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='small'),
        DeclareLaunchArgument(
            'markers_db',
            default_value=[PKG, '/config/markers_db_', map_name, '.yaml']),
        DeclareLaunchArgument('enable_motion', default_value='true'),
        DeclareLaunchArgument('enable_nav', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyTHS1'),

        rsp, delayed_camera, aruco_tf, aruco_notf, esp32, ekf_odom, ekf_map,
        teleop_gw, gui_bridge,
        map_server, planner, controller, recoveries, bt_navigator,
        waypoint_follower, lifecycle_localization, lifecycle_navigation,
    ])
