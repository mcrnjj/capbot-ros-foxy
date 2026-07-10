#!/usr/bin/env python3
"""
object_detector.py
------------------
Deteccion de obstaculos con jetson-inference (detectNet sobre TensorRT, GPU de
la Jetson) + estimacion de distancia MONOCULAR por proyeccion al plano del piso.

POR QUE jetson-inference y no cv2.dnn:
  La OpenCV pip (contrib, cv2.aruco) es CPU-only. detectNet corre en TensorRT
  (GPU) a ~20-25 FPS con SSD-Mobilenet-v2 en la Nano. Este nodo NO captura
  camara: se suscribe a /camera/image_raw (rama cruda del tee de
  csi_camera_node), igual que aruco_localizer. cv2 se usa solo para
  undistortPoints (no toca GStreamer, sin conflicto).

DISTANCIA (plano del piso):
  La camara va HORIZONTAL (camera_joint rpy 0 0 0; el comentario del xacro que
  habla de 45 grados es de un experimento viejo y NO refleja el robot). Un
  obstaculo apoyado en el piso toca el suelo en el borde INFERIOR de su
  bounding box: se toma ese pixel centro-inferior, se retro-proyecta el rayo
  con K (undistort con D) y se interseca con el plano z = ground_z en
  base_frame. Sin asumir tamano del objeto. Requiere:
    - TF base_link -> camera_link_optical (robot_state_publisher, /tf_static)
    - camera.yaml CALIBRADO (la distancia escala con fx/fy)
  Condicionamiento con camara horizontal y baja (~0.13 m sobre el piso):
    - error ~ Z^2/(fy*h) por pixel: cm a 1 m, ~6 cm/px a 2 m -> max_distance 2.0
    - MUY sensible al pitch real: ~1 grado de inclinacion fisica => ~10-15 cm
      de error a 1 m. Calibrar pitch empiricamente (ver procedimiento).
    - rango minimo ~0.3-0.4 m: mas cerca, el borde inferior del bbox queda
      cortado por el borde de la imagen y la distancia se satura ahi.
    - bboxes sobre el horizonte (v < cy) no cortan el piso -> distance_m: null
      (ya manejado).

Topics (pub):
  /detected_objects/poses  geometry_msgs/PoseArray   puntos piso en base_frame
  /detected_objects/info   std_msgs/String (JSON)    clase, conf, bbox, dist
  /detected_objects/cloud  sensor_msgs/PointCloud2   1 punto/obstaculo (nav2)

Parametros (todos con default):
  network ('ssd-mobilenet-v2'), threshold, image_topic, camera_info_topic,
  camera_frame, base_frame, ground_z, max_distance, class_filter,
  publish_cloud, publish_json, min_bbox_height_px, inference_rate_hz
"""

import sys
import json
import time
import struct

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String

import cv2
from scipy.spatial.transform import Rotation as R

from tf2_ros import Buffer, TransformListener

# jetson-inference: API nueva (jetson_inference) o legacy (jetson.inference)
try:
    from jetson_inference import detectNet
    from jetson_utils import cudaFromNumpy
except ImportError:  # instalaciones antiguas
    from jetson.inference import detectNet
    from jetson.utils import cudaFromNumpy


def imgmsg_to_bgr(msg):
    """Convierte sensor_msgs/Image a ndarray BGR SIN cv_bridge.

    Mismo helper que aruco_localizer.py (evitar cv_bridge por choque ABI con
    la OpenCV pip). Soporta bgr8 / rgb8 / mono8 y respeta msg.step."""
    enc = msg.encoding
    h, w = msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("bgr8", "rgb8"):
        rows = buf.reshape(h, msg.step)[:, : w * 3]
        img = rows.reshape(h, w, 3)
        if enc == "rgb8":
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)
    if enc in ("mono8", "8UC1"):
        gray = buf.reshape(h, msg.step)[:, :w]
        return cv2.cvtColor(np.ascontiguousarray(gray), cv2.COLOR_GRAY2BGR)
    raise ValueError("encoding no soportado: %s" % enc)


def make_pointcloud2(points_xyz, frame_id, stamp):
    """Arma un PointCloud2 (x,y,z float32) a mano, sin sensor_msgs_py
    (no existe en Foxy). points_xyz: lista de (x, y, z)."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points_xyz)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points_xyz)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<fff", *p) for p in points_xyz)
    return msg


class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")

        self.declare_parameter("network", "ssd-mobilenet-v2")
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", "camera_link_optical")
        self.declare_parameter("base_frame", "base_link")
        # Altura del piso en base_frame. base_link esta al eje de las ruedas
        # (r = 0.035 m en robot_core.xacro) => piso en z = -0.035. VERIFICAR:
        #   ros2 run tf2_ros tf2_echo base_link camera_link_optical
        self.declare_parameter("ground_z", -0.035)
        # Descarta detecciones proyectadas mas alla de esto (m, en el piso).
        # Con camara horizontal a ~0.13 m el error crece ~Z^2: >2 m no es fiable.
        self.declare_parameter("max_distance", 2.0)
        # Clases COCO permitidas (vacio = todas), p.ej. ['person','chair'].
        self.declare_parameter("class_filter", [""])
        self.declare_parameter("publish_cloud", True)
        self.declare_parameter("publish_json", True)
        # bboxes muy chatos suelen ser falsos positivos lejanos.
        self.declare_parameter("min_bbox_height_px", 20)
        # Tasa MAXIMA de inferencia (Hz); 0 = sin limite (cada frame). Bajarla
        # ahorra energia: la GPU solo trabaja en los frames que pasan el
        # throttle. A nav2 le sobra con 3 Hz (el planner replanifica a ~1 Hz y
        # el costmap local actualiza a 5 Hz).
        self.declare_parameter("inference_rate_hz", 3.0)

        gp = lambda n: self.get_parameter(n).get_parameter_value()
        self.camera_frame = gp("camera_frame").string_value
        self.base_frame = gp("base_frame").string_value
        self.ground_z = gp("ground_z").double_value
        self.max_distance = gp("max_distance").double_value
        self.min_bbox_h = gp("min_bbox_height_px").integer_value
        self.class_filter = [c for c in gp("class_filter").string_array_value if c]
        self.do_cloud = gp("publish_cloud").bool_value
        self.do_json = gp("publish_json").bool_value
        rate = gp("inference_rate_hz").double_value
        self._inf_period = (1.0 / rate) if rate > 0 else 0.0
        self._last_inf = 0.0

        network = gp("network").string_value
        threshold = gp("threshold").double_value

        # Cargar la red. La PRIMERA vez TensorRT construye el engine
        # (~3-5 min en la Nano); despues queda cacheado junto al modelo.
        self.get_logger().info(
            "Cargando detectNet '%s' (si es la 1ra vez, TensorRT tarda varios "
            "minutos construyendo el engine)..." % network)
        self.net = detectNet(network, threshold=threshold)
        self.get_logger().info("detectNet listo.")

        self.K = None
        self.D = None

        # TF estatico base -> camara (lo publica robot_state_publisher).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.T_base_cam = None  # (R 3x3, t 3) cacheado; el joint es fijo.

        self.pub_poses = self.create_publisher(PoseArray, "/detected_objects/poses", 5)
        self.pub_info = self.create_publisher(String, "/detected_objects/info", 5)
        self.pub_cloud = self.create_publisher(PointCloud2, "/detected_objects/cloud", 5)

        # QoS: best-effort, depth 1 -> siempre el frame MAS RECIENTE. La
        # inferencia (~40-50 ms) es mas lenta que la camara (30 fps); con
        # depth 1 se descartan frames viejos en vez de acumular lag.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            CameraInfo, gp("camera_info_topic").string_value, self.cam_info_cb, 10)
        self.create_subscription(
            Image, gp("image_topic").string_value, self.image_cb, qos)

        self._busy = False
        self._frames = 0
        self._log_timer = self.create_timer(10.0, self._log_rate)

    # ------------------------------------------------------------------ utils

    def cam_info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64).reshape(-1)
            self.get_logger().info("CameraInfo recibido (fx=%.1f fy=%.1f)."
                                   % (self.K[0, 0], self.K[1, 1]))

    def lookup_base_cam(self):
        """Cachea base_frame -> camera_frame (joint fijo, /tf_static)."""
        if self.T_base_cam is not None:
            return self.T_base_cam
        try:
            tr = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, rclpy.time.Time())
        except Exception as e:  # aun no llega el static tf
            self.get_logger().warn("TF %s->%s no disponible: %s"
                                   % (self.base_frame, self.camera_frame, e),
                                   throttle_duration_sec=5.0)
            return None
        q = tr.transform.rotation
        t = tr.transform.translation
        Rm = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        self.T_base_cam = (Rm, np.array([t.x, t.y, t.z]))
        self.get_logger().info("TF base->camara cacheado (cam en z=%.3f m)." % t.z)
        return self.T_base_cam

    def pixel_to_ground(self, u, v):
        """Retro-proyecta el pixel (u,v) e interseca con z = ground_z en
        base_frame. Devuelve (x, y, z) o None si el rayo no corta el piso
        hacia adelante (p.ej. bbox cortado por el borde superior)."""
        T = self.lookup_base_cam()
        if T is None or self.K is None:
            return None
        Rm, t = T
        # undistortPoints -> coordenadas normalizadas (ya sin distorsion)
        pt = np.array([[[float(u), float(v)]]], dtype=np.float64)
        norm = cv2.undistortPoints(pt, self.K, self.D).reshape(2)
        d_cam = np.array([norm[0], norm[1], 1.0])
        d_base = Rm @ d_cam
        if d_base[2] > -1e-6:  # el rayo no apunta hacia abajo
            return None
        s = (self.ground_z - t[2]) / d_base[2]
        if s <= 0:
            return None
        return t + s * d_base

    # ------------------------------------------------------------------ main

    def image_cb(self, msg):
        if self._busy:
            return
        # Throttle ANTES de tocar el frame: los que se descartan no pagan ni la
        # conversion numpy ni la copia H->D ni la GPU (ese es el ahorro).
        now = time.monotonic()
        if self._inf_period > 0.0 and (now - self._last_inf) < self._inf_period:
            return
        self._last_inf = now
        self._busy = True
        try:
            self.process(msg)
        except Exception as e:
            self.get_logger().error("process(): %s" % e)
        finally:
            self._busy = False

    def process(self, msg):
        bgr = imgmsg_to_bgr(msg)
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        cuda_img = cudaFromNumpy(rgb)          # copia H->D, formato rgb8
        dets = self.net.Detect(cuda_img, overlay="none")
        self._frames += 1

        h, w = bgr.shape[:2]
        poses = PoseArray()
        poses.header.stamp = msg.header.stamp
        poses.header.frame_id = self.base_frame
        cloud_pts = []
        info = []

        for d in dets:
            cls = self.net.GetClassDesc(d.ClassID)
            if self.class_filter and cls not in self.class_filter:
                continue
            if (d.Bottom - d.Top) < self.min_bbox_h:
                continue
            # pixel centro-inferior del bbox = punto de contacto con el piso
            u = min(max(d.Center[0], 0.0), w - 1.0)
            v = min(d.Bottom, h - 1.0)
            p = self.pixel_to_ground(u, v)
            entry = {"class": cls, "confidence": round(float(d.Confidence), 3),
                     "bbox": [round(d.Left, 1), round(d.Top, 1),
                              round(d.Right, 1), round(d.Bottom, 1)]}
            if p is not None:
                dist = float(np.hypot(p[0], p[1]))
                if self.max_distance > 0 and dist > self.max_distance:
                    continue
                entry.update({"x": round(float(p[0]), 3),
                              "y": round(float(p[1]), 3),
                              "distance_m": round(dist, 3)})
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = \
                    float(p[0]), float(p[1]), float(p[2])
                pose.orientation.w = 1.0
                poses.poses.append(pose)
                cloud_pts.append((float(p[0]), float(p[1]), float(p[2])))
            else:
                entry["distance_m"] = None  # sin TF/K o bbox cortado arriba
            info.append(entry)

        self.pub_poses.publish(poses)
        if self.do_cloud:
            self.pub_cloud.publish(
                make_pointcloud2(cloud_pts, self.base_frame, msg.header.stamp))
        if self.do_json:
            s = String(); s.data = json.dumps(info)
            self.pub_info.publish(s)

    def _log_rate(self):
        self.get_logger().info("inferencia: %.1f FPS" % (self._frames / 10.0))
        self._frames = 0


def main():
    rclpy.init()
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
