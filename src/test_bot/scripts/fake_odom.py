#!/usr/bin/env python3
"""
fake_odom.py - simulador cinematico diff-drive para testear nav2 SIN el ESP32.

Reemplaza a esp32_serial_bridge + EKF: consume /cmd_vel (lo que produce nav2),
integra una pose 2D y publica:
  - odometria en <odom_topic> (default /odometry/filtered_odom, el que lee nav2)
    y tambien en /odom (por generalidad),
  - TF odom -> base_link,
  - (opcional) TF estatico map -> odom = identidad, para que nav2 tenga
    localizacion trivial sin ArUco (map = odom).

Asi el lazo de nav2 se cierra en software: goal -> plan -> /cmd_vel -> el robot
"se mueve" (se integra la pose) -> TF/odom -> nav2 ve el avance -> llega al goal.
Sin hardware ni firmware.
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class FakeOdom(Node):
    def __init__(self):
        super().__init__("fake_odom")

        self.declare_parameter("odom_topic", "/odometry/filtered_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_map_odom", True)   # map=odom identidad
        self.declare_parameter("max_linear_speed", 0.5)    # clamp de seguridad
        self.declare_parameter("max_angular_speed", 2.0)

        gp = self.get_parameter
        odom_topic = gp("odom_topic").value
        self.odom_frame = gp("odom_frame").value
        self.base_frame = gp("base_frame").value
        self.map_frame = gp("map_frame").value
        self.publish_tf = bool(gp("publish_tf").value)
        self.max_lin = float(gp("max_linear_speed").value)
        self.max_ang = float(gp("max_angular_speed").value)
        rate = float(gp("rate").value)
        self.dt = 1.0 / max(1.0, rate)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.odom_pub2 = self.create_publisher(Odometry, "/odom", 10)
        self.tf_b = TransformBroadcaster(self)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_timer(self.dt, self._step)

        if bool(gp("publish_map_odom").value):
            self.static_b = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.odom_frame
            t.transform.rotation.w = 1.0   # identidad
            self.static_b.sendTransform(t)

        self.get_logger().info(
            "fake_odom: /cmd_vel -> pose integrada -> %s + /odom + TF odom->base_link. "
            "map=odom (identidad). Simulador para nav2 SIN ESP32." % odom_topic)

    def _on_cmd(self, msg):
        self.v = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        self.w = max(-self.max_ang, min(self.max_ang, msg.angular.z))

    def _step(self):
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt
        self.th += self.w * self.dt

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(self.th)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w
        od.pose.covariance[0] = 0.02
        od.pose.covariance[7] = 0.02
        od.pose.covariance[35] = 0.04
        od.twist.covariance[0] = 0.02
        od.twist.covariance[35] = 0.04
        self.odom_pub.publish(od)
        self.odom_pub2.publish(od)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_b.sendTransform(t)


def main():
    rclpy.init()
    node = FakeOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
