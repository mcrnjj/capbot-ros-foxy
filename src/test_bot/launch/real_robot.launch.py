"""
real_robot.launch.py - bringup del robot REAL en la Jetson (ROS2 Foxy, Docker).

FASE 2 (este archivo): localizacion + odometria + EKF
  - robot_state_publisher (robot_real.urdf.xacro, sin lidar/gazebo)
  - csi_camera_node       (camara CSI -> /camera/image_raw + video H264 a la GUI)
  - aruco_localizer       (-> /aruco_pose)
  - esp32_serial_bridge   (serie ESP32 -> /odom ; /cmd_vel -> VEL_CMD)
  - ekf x2 (robot_localization): local (odom->base_link) y global (map->odom)

Flag enable_motion (default true):
  - true  : Fase 2. Corren esp32_bridge + EKF; el TF lo dan los EKF
            (aruco con publish_tf:=false).
  - false : Fase 1 (camara-only, sin hardware). aruco publica TF map->base_link
            directo y no se lanzan esp32_bridge ni EKF.

Fase 3 (se agrega aqui): nav2 + map_server + gui_bridge_node (flag enable_nav).

Uso:
  ros2 launch test_bot real_robot.launch.py
  ros2 launch test_bot real_robot.launch.py enable_motion:=false   # solo camara/aruco
  ros2 launch test_bot real_robot.launch.py map_name:=large serial_port:=/dev/ttyUSB0
  HOST_IP=192.168.1.10 ros2 launch test_bot real_robot.launch.py   # video a la GUI
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

PKG = get_package_share_directory('test_bot')


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')
    markers_db = LaunchConfiguration('markers_db')
    enable_motion = LaunchConfiguration('enable_motion')
    serial_port = LaunchConfiguration('serial_port')

    camera_info = 'file://' + os.path.join(PKG, 'config', 'camera.yaml')
    ekf_params = os.path.join(PKG, 'config', 'ekf_real.yaml')

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
        'max_distance': 2.0,
        # 27 es muy laxo a proposito: con camera.yaml SIN calibrar el error de
        # reproyeccion es alto. RECALIBRAR camera.yaml y bajar a ~3.0.
        'max_reproj_error_px': 27.0,
        'min_marker_area_px': 200.0,
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
            'max_linear_speed': 0.3,    # m/s; ajustar a la velocidad real
            'max_angular_speed': 2.0,   # rad/s; idem
            'cmd_vel_timeout': 0.5,
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
                     'base_frame': 'base_link', 'odom_frame': 'odom'}],
        condition=IfCondition(enable_motion),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='small'),
        DeclareLaunchArgument(
            'markers_db',
            default_value=[PKG, '/config/markers_db_', map_name, '.yaml']),
        DeclareLaunchArgument('enable_motion', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyTHS1'),

        rsp, camera, aruco_tf, aruco_notf, esp32, ekf_odom, ekf_map,
        teleop_gw, gui_bridge,
    ])
