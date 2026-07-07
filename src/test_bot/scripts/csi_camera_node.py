#!/usr/bin/env python3
"""
csi_camera_node.py
------------------
Nodo de camara CSI (IMX219 / Raspberry Cam v2) para Jetson Nano en ROS2 Foxy.

POR QUE PyGObject y no cv2:
  En la Jetson la captura CSI pasa por el ISP de NVIDIA (Argus) via GStreamer
  (`nvarguscamerasrc`). El conflicto clasico es que la OpenCV con `cv2.aruco`
  (contrib) NO suele traer soporte GStreamer y viceversa. Para evitarlo, este
  nodo captura con **GStreamer puro via PyGObject** (gi.repository.Gst) y arma
  el `sensor_msgs/Image` a mano (sin cv2 ni cv_bridge). Asi `aruco_localizer`
  puede usar una OpenCV-contrib sin gstreamer, y aqui no hace falta cv2.

PIPELINE con `tee` (una sola captura sirve a dos consumidores):
  nvarguscamerasrc
    -> tee
        |-- rama CRUDA  -> appsink BGR  -> /camera/image_raw (para ArUco)
        |-- rama VIDEO  -> nvv4l2h264enc -> RTP/UDP al PC (capbot-host video_dock)

  La rama de video reproduce el mismo pipeline que capbot-jetson-bridge
  (net/video_pipeline.py) para que la GUI funcione sin cambios. Solo se activa
  si hay HOST_IP (parametro `host_ip` o variable de entorno HOST_IP).

Topics:
  /camera/image_raw    sensor_msgs/Image      (bgr8)
  /camera/camera_info  sensor_msgs/CameraInfo (de un yaml estilo camera_calibration)

Parametros (todos con default):
  sensor_id, capture_width/height, output_width/height, framerate, flip_method,
  frame_id, camera_info_url, publish_rate,
  enable_video (bool), host_ip (str), video_port (int), video_bitrate_kbps (int)
"""

import os
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # noqa: E402


def build_pipeline(sid, cw, ch, ow, oh, fps, flip, enable_video,
                   host, port, bitrate_kbps):
    """Arma el string del pipeline GStreamer.

    Siempre incluye la rama cruda (appsink BGR). Si enable_video y host, agrega
    la rama H264/RTP hacia el PC. Sin video, no se usa `tee` (mas simple)."""
    raw_branch = (
        "nvvidconv flip-method={flip} ! "
        "video/x-raw,width={ow},height={oh},format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=sink emit-signals=true drop=true max-buffers=1 sync=false"
    ).format(flip=flip, ow=ow, oh=oh)

    src = (
        "nvarguscamerasrc sensor-id={sid} ! "
        "video/x-raw(memory:NVMM),width={cw},height={ch},"
        "framerate={fps}/1,format=NV12 ! "
    ).format(sid=sid, cw=cw, ch=ch, fps=fps)

    if not (enable_video and host):
        return src + raw_branch

    video_branch = (
        "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
        "nvv4l2h264enc insert-sps-pps=true iframeinterval=15 idrinterval=15 "
        "maxperf-enable=0 preset-level=1 bitrate={br} control-rate=1 ! "
        "h264parse config-interval=1 ! "
        "rtph264pay pt=96 config-interval=1 mtu=1400 ! "
        "udpsink host={host} port={port} sync=false async=false"
    ).format(br=int(bitrate_kbps) * 1000, host=host, port=port)

    # queue leaky=downstream (descarta frames viejos en vez de acumular) -> baja
    # latencia. Sin esto los queue bufferean hasta ~1 s y se ve el video con delay.
    q = "queue leaky=2 max-size-buffers=2 max-size-bytes=0 max-size-time=0"

    return (
        src + "tee name=t "
        "t. ! " + q + " ! " + raw_branch + " "
        "t. ! " + q + " ! " + video_branch
    )


def load_camera_info(path, default_w, default_h, frame_id):
    """Carga un yaml estilo camera_calibration en un CameraInfo. Devuelve
    (CameraInfo, ok). Si falla, CameraInfo minimo y ok=False (se avisa)."""
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = default_w
    info.height = default_h

    if path.startswith("file://"):
        path = path[len("file://"):]
    if not path or not os.path.isfile(path):
        return info, False

    import yaml
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    info.width = int(data.get("image_width", default_w))
    info.height = int(data.get("image_height", default_h))
    info.distortion_model = data.get("distortion_model", "plumb_bob")

    def _mat(key):
        node = data.get(key, {})
        return [float(x) for x in node.get("data", [])]

    k = _mat("camera_matrix")
    d = _mat("distortion_coefficients")
    r = _mat("rectification_matrix")
    p = _mat("projection_matrix")
    if len(k) == 9:
        info.k = k
    if d:
        info.d = d
    if len(r) == 9:
        info.r = r
    if len(p) == 12:
        info.p = p
    return info, True


class CsiCameraNode(Node):
    def __init__(self):
        super().__init__("csi_camera_node")

        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("capture_width", 1280)
        self.declare_parameter("capture_height", 720)
        self.declare_parameter("output_width", 640)
        self.declare_parameter("output_height", 480)
        self.declare_parameter("framerate", 30)
        self.declare_parameter("flip_method", 0)
        self.declare_parameter("frame_id", "camera_link_optical")
        self.declare_parameter("camera_info_url", "")
        # Video hacia la GUI (capbot-host). host_ip vacio => usa env HOST_IP.
        self.declare_parameter("enable_video", True)
        self.declare_parameter("host_ip", "")
        self.declare_parameter("video_port", 5000)
        self.declare_parameter("video_bitrate_kbps", 4000)
        # Tasa de publicacion de /camera/image_raw (rama cruda -> aruco). Mas baja
        # que 'framerate' para no saturar la CPU; el video H264 a la GUI no se afecta.
        self.declare_parameter("publish_rate", 12.0)

        gp = self.get_parameter
        sid = gp("sensor_id").value
        cw = gp("capture_width").value
        ch = gp("capture_height").value
        self.ow = gp("output_width").value
        self.oh = gp("output_height").value
        fps = gp("framerate").value
        flip = gp("flip_method").value
        self.frame_id = gp("frame_id").value
        info_url = gp("camera_info_url").value
        enable_video = bool(gp("enable_video").value)
        host = gp("host_ip").value or os.environ.get("HOST_IP", "")
        port = gp("video_port").value
        bitrate = gp("video_bitrate_kbps").value
        pub_rate = float(gp("publish_rate").value)
        self._pub_period = (1.0 / pub_rate) if pub_rate > 0 else 0.0
        self._last_pub = 0.0

        self.camera_info, ok = load_camera_info(info_url, self.ow, self.oh, self.frame_id)
        if not ok:
            self.get_logger().warn(
                "Sin calibracion valida en '%s'. ArUco necesita K real; "
                "publicando CameraInfo casi vacio." % info_url)

        self.image_pub = self.create_publisher(Image, "/camera/image_raw",
                                               qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

        Gst.init(None)
        pipeline_str = build_pipeline(sid, cw, ch, self.ow, self.oh, fps, flip,
                                      enable_video, host, port, bitrate)
        self.get_logger().info("GStreamer: %s" % pipeline_str)

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as exc:
            self.get_logger().fatal("parse_launch fallo: %s" % exc)
            sys.exit(1)

        sink = self.pipeline.get_by_name("sink")
        sink.connect("new-sample", self._on_sample)

        gbus = self.pipeline.get_bus()
        gbus.add_signal_watch()
        gbus.connect("message::error", self._on_gst_error)

        self.pipeline.set_state(Gst.State.PLAYING)

        # MainLoop de GLib en un hilo daemon (la captura corre en hilos de GStreamer)
        self._glib_loop = GLib.MainLoop()
        self._glib_thread = threading.Thread(target=self._glib_loop.run, daemon=True)
        self._glib_thread.start()

        if enable_video and host:
            self.get_logger().info("Video H264/RTP -> %s:%d" % (host, port))
        else:
            self.get_logger().warn(
                "Video a la GUI DESACTIVADO (enable_video=%s, host_ip='%s'). "
                "Exporta HOST_IP o pasa host_ip para el video_dock." % (enable_video, host))

        self.get_logger().info(
            "CSI lista: %dx%d, frame=%s" % (self.ow, self.oh, self.frame_id))

    def _on_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        # Throttle de la rama cruda: publicar a publish_rate (a aruco le sobra) para
        # no saturar la CPU de la Nano. El video H264 a la GUI NO se afecta (es otra
        # rama del pipeline). Igual se pulle el sample para liberar el buffer.
        now = time.monotonic()
        if self._pub_period > 0.0 and (now - self._last_pub) < self._pub_period:
            return Gst.FlowReturn.OK
        self._last_pub = now

        buf = sample.get_buffer()
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        try:
            data = bytes(mapinfo.data)  # BGR empaquetado (h*w*3)
        finally:
            buf.unmap(mapinfo)

        stamp = self.get_clock().now().to_msg()

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = self.oh
        msg.width = self.ow
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = self.ow * 3
        msg.data = data
        self.image_pub.publish(msg)

        self.camera_info.header.stamp = stamp
        self.info_pub.publish(self.camera_info)
        return Gst.FlowReturn.OK

    def _on_gst_error(self, _bus, gmsg):
        err, dbg = gmsg.parse_error()
        self.get_logger().error("GStreamer: %s (%s)" % (err.message, dbg))

    def destroy_node(self):
        try:
            self.pipeline.set_state(Gst.State.NULL)
            if self._glib_loop.is_running():
                self._glib_loop.quit()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CsiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
