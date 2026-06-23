"""
real_robot.launch.py - bringup del robot REAL en la Jetson (ROS2 Foxy, Docker).

FASE 1 (este archivo): localizacion/vision
  - robot_state_publisher (robot_real.urdf.xacro, sin lidar/gazebo)
  - joint_state_publisher (ceros para las ruedas; completa el TF tree)
  - csi_camera_node       (camara CSI -> /camera/image_raw + video H264 a la GUI)
  - aruco_localizer       (-> /aruco_pose, y en Fase 1 publica TF map->base_link)

Fases siguientes (se agregan aqui):
  - Fase 2: esp32_serial_bridge (/odom) + ekf x2 + teleop_gateway + cmd_mux
            (aruco pasa a publish_tf:=false; el TF lo da el EKF).
  - Fase 3: nav2 + map_server + gui_bridge_node (flag enable_nav).

Uso:
  ros2 launch test_bot real_robot.launch.py
  ros2 launch test_bot real_robot.launch.py map_name:=large
  HOST_IP=192.168.1.10 ros2 launch test_bot real_robot.launch.py   # video a la GUI
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
    markers_db = LaunchConfiguration('markers_db')

    camera_info = 'file://' + os.path.join(PKG, 'config', 'camera.yaml')

    xacro_file = os.path.join(PKG, 'description', 'robot_real.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    # Nota: no se usa joint_state_publisher. Los frames que necesita ArUco
    # (base_link -> camera_link_optical) son joints FIJOS y robot_state_publisher
    # los publica en /tf_static sin /joint_states. (Las ruedas continuas no se
    # publican en TF, pero no hacen falta para localizacion.)

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

    aruco = Node(
        package='test_bot', executable='aruco_localizer',
        name='aruco_localizer', output='screen',
        parameters=[{
            'use_sim_time': False,
            'markers_db': markers_db,
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'camera_frame': 'camera_link_optical',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'publish_tf': True,     # Fase 1: ArUco da el TF map->base_link (sin EKF).
            'max_distance': 2.0,
            # 6.0 es laxo a proposito: con camera.yaml SIN calibrar el error de
            # reproyeccion ronda 3-4px. RECALIBRAR camera.yaml y bajar a ~3.0.
            'max_reproj_error_px': 6.0,
            'min_marker_area_px': 200.0,
            'filter_window': 1,
            'ambiguity_ratio_threshold': 1.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='small'),
        DeclareLaunchArgument(
            'markers_db',
            default_value=[PKG, '/config/markers_db_', map_name, '.yaml']),

        rsp, camera, aruco,
    ])
