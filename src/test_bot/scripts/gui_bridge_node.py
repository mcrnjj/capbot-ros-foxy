#!/usr/bin/env python3
"""
gui_bridge_node.py
------------------
Puente WebSocket entre la GUI (capbot-host, network/nav_client.py) y nav2.
Expone WS 8766 (NAV.gui_bridge_port). Protocolo JSON:

  host -> ROS:   {"type":"goal", "x":.., "y":.., "yaw":..}   (frame map)
                 {"type":"cancel"}
  ROS  -> host:  {"type":"pose", "valid":true, "x":.., "y":.., "yaw":..}
                 {"type":"nav_status", "state":"..", "distance_remaining":..}

La POSE se publica SIEMPRE (~10 Hz) desde TF, independiente del modo (manual o
autonomo): asi el mapa de la GUI muestra el robot moverse como en rviz2. Se usa
TF map->base_link; si no hay (sin localizacion global), cae a odom->base_link.

El goal se reenvia a la accion nav2 NavigateToPose; el nav_status sale de su
feedback/resultado.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import tf2_ros

try:
    import asyncio
    import json
    import websockets
except ImportError:
    websockets = None


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class GuiBridge(Node):
    def __init__(self):
        super().__init__("gui_bridge_node")

        self.declare_parameter("ws_port", 8766)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("pose_rate", 10.0)
        self.declare_parameter("map_name", "small")
        gp = self.get_parameter
        self.ws_port = int(gp("ws_port").value)
        self.map_frame = gp("map_frame").value
        self.base_frame = gp("base_frame").value
        self.odom_frame = gp("odom_frame").value
        self.map_name = str(gp("map_name").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._nav_ready = False
        # wait_for_server blocks with time.sleep() internally — run in a thread
        # so the ROS spin loop (needed for DDS discovery) is never blocked.
        threading.Thread(target=self._wait_for_nav, daemon=True).start()

        # Estado compartido con el hilo WS
        self._lock = threading.Lock()
        self._latest_pose = {"type": "pose", "valid": False, "x": 0.0, "y": 0.0, "yaw": 0.0}
        self._latest_status = None
        self._goal_req = None        # (x, y, yaw) pendiente de enviar
        self._cancel_req = False
        self._goal_handle = None

        rate = float(gp("pose_rate").value)
        self.create_timer(1.0 / max(1.0, rate), self._tick)

        self._running = True
        if websockets is not None:
            self._ws_thread = threading.Thread(target=self._ws_loop, daemon=True)
            self._ws_thread.start()
        else:
            self.get_logger().warn("websockets no instalado; WS de navegacion deshabilitado.")

        self.get_logger().info(
            "gui_bridge_node: WS %d (goals<-, pose/nav_status->). Pose siempre por TF." % self.ws_port)

    def _wait_for_nav(self):
        if self._action.wait_for_server(timeout_sec=60.0):
            self._nav_ready = True
            self.get_logger().info("navigate_to_pose: servidor disponible.")
        else:
            self.get_logger().warn("navigate_to_pose: servidor no encontrado en 60 s.")

    # ----------------------- timer ROS -----------------------
    def _tick(self):
        # 1) Pose desde TF (map->base, fallback odom->base).
        pose = self._lookup_pose()
        with self._lock:
            self._latest_pose = pose
            goal_req = self._goal_req
            self._goal_req = None
            cancel = self._cancel_req
            self._cancel_req = False

        # 2) Enviar goal pendiente.
        if goal_req is not None:
            self._send_goal(*goal_req)
        if cancel and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _lookup_pose(self):
        for target, source in ((self.map_frame, self.base_frame),
                               (self.odom_frame, self.base_frame)):
            try:
                tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                return {"type": "pose", "valid": True,
                        "x": float(t.x), "y": float(t.y),
                        "yaw": float(quat_to_yaw(q.x, q.y, q.z, q.w))}
            except Exception:
                continue
        return {"type": "pose", "valid": False, "x": 0.0, "y": 0.0, "yaw": 0.0}

    # ----------------------- accion nav2 -----------------------
    def _send_goal(self, x, y, yaw):
        if not self._nav_ready:
            self._set_status("rejected")
            self.get_logger().warn("navigate_to_pose no disponible (nav2 corriendo?).")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        fut = self._action.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self._set_status("rejected")
            return
        self._goal_handle = handle
        self._set_status("accepted")
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        try:
            dist = float(feedback_msg.feedback.distance_remaining)
        except Exception:
            dist = 0.0
        self._set_status("active", dist)

    def _on_result(self, future):
        status = future.result().status
        mapping = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled",
        }
        self._set_status(mapping.get(status, "aborted"))
        self._goal_handle = None

    def _set_status(self, state, dist=None):
        msg = {"type": "nav_status", "state": state}
        if dist is not None:
            msg["distance_remaining"] = dist
        with self._lock:
            self._latest_status = msg

    # ----------------------- WS -----------------------
    def _ws_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        async def handler(ws, path=None):
            # Announce the active map so the host can auto-select it
            try:
                await ws.send(json.dumps({"type": "map_name", "name": self.map_name}))
            except Exception:
                return
            sender = asyncio.ensure_future(self._ws_send(ws))
            try:
                async for raw in ws:
                    self._handle_ws_msg(raw)
            except Exception:
                pass
            finally:
                sender.cancel()

        try:
            server = websockets.serve(handler, "0.0.0.0", self.ws_port)
            loop.run_until_complete(server)
            loop.run_forever()
        except Exception as e:
            self.get_logger().error("WS navegacion fallo: %s" % e)

    async def _ws_send(self, ws):
        last_status = None
        while self._running:
            with self._lock:
                pose = dict(self._latest_pose)
                status = self._latest_status
            try:
                await ws.send(json.dumps(pose))
                if status is not None and status != last_status:
                    await ws.send(json.dumps(status))
                    last_status = status
            except Exception:
                return
            await asyncio.sleep(0.1)   # 10 Hz

    def _handle_ws_msg(self, raw):
        try:
            if isinstance(raw, bytes):
                raw = raw.decode("utf-8")
            data = json.loads(raw)
        except (UnicodeDecodeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        if data.get("type") == "goal":
            with self._lock:
                self._goal_req = (float(data.get("x", 0.0)),
                                  float(data.get("y", 0.0)),
                                  float(data.get("yaw", 0.0)))
        elif data.get("type") == "cancel":
            with self._lock:
                self._cancel_req = True

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main():
    rclpy.init()
    node = GuiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
