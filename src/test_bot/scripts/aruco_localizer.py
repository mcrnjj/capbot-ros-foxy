#!/usr/bin/env python3
"""
aruco_localizer.py  (v3 - filtros y robustez)
---------------------------------------------
Mejoras respecto a la versión previa:
  1. solvePnPGeneric con SOLVEPNP_IPPE_SQUARE: devuelve las DOS soluciones
     ambiguas y se elige la de menor error de reproyección (resuelve el flip).
  2. Cálculo y log del error de reproyección por marcador (px). Permite
     diagnosticar problemas de calibración o tamaño.
  3. Rechazo de detecciones por distancia (--max_distance) y por error de
     reproyección excesivo (--max_reproj_error_px).
  4. Ponderación de marcadores múltiples por área del marcador en imagen
     (los más grandes pesan más en el promedio).
  5. Filtro temporal (media móvil con ventana configurable) sobre la pose
     final en map. Limpia jitter sin introducir lag agresivo.
  6. Publica además el error medio de reproyección como diagnóstico en
     /aruco_pose/mean_reproj_error.

Parámetros nuevos:
  max_distance              : descarta marcadores con ||tvec|| mayor (m). 0 = sin limite. Def 1.5
  max_reproj_error_px       : descarta detecciones con error mayor (px). Def 3.0
  filter_window             : tamaño de la ventana del filtro temporal. 1 = sin filtro. Def 5
  min_marker_area_px        : descarta marcadores con area menor (px2). Def 400
  ambiguity_ratio_threshold : si err2 < err1 * ratio, alerta de pose ambigua. Def 1.5
"""

import sys
import collections
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import Float32

import cv2
from scipy.spatial.transform import Rotation as R

import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


# -----------------------------------------------------------------------------
# Helpers de detección y estimación
# -----------------------------------------------------------------------------

def imgmsg_to_bgr(msg):
    """Convierte sensor_msgs/Image a un ndarray BGR SIN cv_bridge.

    Se evita cv_bridge a proposito: en la Jetson cv_bridge linkea la OpenCV del
    sistema, que puede chocar (ABI) con la OpenCV-contrib que usamos para aruco.
    Soporta bgr8 / rgb8 / mono8 y respeta el row stride (msg.step)."""
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


def detect_markers(gray, aruco_dict, params):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)


def estimate_marker_pose_dual(corners, marker_size, K, D):
    """
    Estima la pose del marcador usando solvePnPGeneric con IPPE_SQUARE.
    Devuelve hasta 2 soluciones (rvec, tvec, error_reproj_px) ordenadas por
    error de reproyección ascendente. Permite resolver el flip de pose
    plana eligiendo siempre la solución más consistente.
    """
    half = marker_size / 2.0
    obj_pts = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)
    img_pts = corners.reshape(-1, 2).astype(np.float32)

    flag = (cv2.SOLVEPNP_IPPE_SQUARE
            if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE")
            else cv2.SOLVEPNP_ITERATIVE)

    if hasattr(cv2, "solvePnPGeneric"):
        ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj_pts, img_pts, K, D, flags=flag)
        if not ok or len(rvecs) == 0:
            return []
        results = []
        for rvec, tvec in zip(rvecs, tvecs):
            err = _reprojection_error(obj_pts, img_pts, rvec, tvec, K, D)
            results.append((rvec, tvec, err))
        results.sort(key=lambda r: r[2])
        return results

    # Fallback (OpenCV antiguo): una sola solución
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=flag)
    if not ok:
        return []
    err = _reprojection_error(obj_pts, img_pts, rvec, tvec, K, D)
    return [(rvec, tvec, err)]


def _reprojection_error(obj_pts, img_pts, rvec, tvec, K, D):
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)
    proj = proj.reshape(-1, 2)
    return float(np.linalg.norm(proj - img_pts, axis=1).mean())


def marker_area_px(corners):
    """Area del polígono del marcador en la imagen (px^2). Shoelace."""
    pts = corners.reshape(-1, 2)
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def to_matrix(translation, quat_xyzw):
    T = np.eye(4)
    T[:3, 3] = translation
    T[:3, :3] = R.from_quat(quat_xyzw).as_matrix()
    return T


def matrix_to_translation_quat(T):
    q = R.from_matrix(T[:3, :3]).as_quat()
    return T[:3, 3], q


def average_poses(matrices, weights=None):
    """Promedio ponderado de poses 4x4. Traslación lineal, rotación con quat
    averaging y hemisferio consistente."""
    if len(matrices) == 1:
        return matrices[0]
    if weights is None:
        weights = np.ones(len(matrices))
    weights = np.asarray(weights, dtype=float)
    weights = weights / weights.sum()

    T_avg = np.eye(4)
    T_avg[:3, 3] = np.sum(
        [w * T[:3, 3] for w, T in zip(weights, matrices)], axis=0
    )

    quats = np.array([R.from_matrix(T[:3, :3]).as_quat() for T in matrices])
    for i in range(1, len(quats)):
        if np.dot(quats[0], quats[i]) < 0:
            quats[i] = -quats[i]
    q = (weights[:, None] * quats).sum(axis=0)
    q /= np.linalg.norm(q)
    T_avg[:3, :3] = R.from_quat(q).as_matrix()
    return T_avg


# -----------------------------------------------------------------------------
# Filtro temporal sobre pose en map
# -----------------------------------------------------------------------------

class PoseFilter:
    """Media móvil sobre las últimas N poses en map (traslación + quat)."""

    def __init__(self, window_size: int):
        self.window = max(1, int(window_size))
        self.buffer = collections.deque(maxlen=self.window)

    def update(self, T):
        self.buffer.append(T)
        if len(self.buffer) == 1:
            return T
        return average_poses(list(self.buffer))


# -----------------------------------------------------------------------------
# Nodo
# -----------------------------------------------------------------------------

class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__("aruco_localizer")

        self.declare_parameter("markers_db", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", "camera_link_optical")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_tf", False)

        # Parametros de robustez:
        self.declare_parameter("max_distance", 1.5)
        self.declare_parameter("max_reproj_error_px", 3.0)
        self.declare_parameter("filter_window", 1)
        self.declare_parameter("min_marker_area_px", 150.0)
        self.declare_parameter("ambiguity_ratio_threshold", 1.5)

        # Covarianza DINAMICA de /aruco_pose: base * (1 + cov_reproj_scale*err_px)
        # * (1 + cov_dist_scale*d^2) / n_marcadores. Una vista cercana y nitida
        # corrige fuerte el EKF de map; una lejana/borrosa apenas lo mueve.
        # Con la camara SIN calibrar el err_px es alto y la covarianza crece
        # sola (auto-desconfianza); al recalibrar camera.yaml se aprieta sola.
        self.declare_parameter("base_cov_xy", 0.02)        # m^2 (var x e y)
        self.declare_parameter("base_cov_yaw", 0.05)       # rad^2
        self.declare_parameter("cov_reproj_scale", 0.5)    # por px de err medio
        self.declare_parameter("cov_dist_scale", 1.0)      # por m^2 de distancia

        db_path = self.get_parameter("markers_db").get_parameter_value().string_value
        if not db_path:
            self.get_logger().fatal("Parametro 'markers_db' obligatorio.")
            sys.exit(1)

        with open(db_path, "r") as f:
            db = yaml.safe_load(f)

        dict_name = db.get("aruco_dict", "DICT_5X5_250")
        self.marker_size = float(db.get("marker_size", 0.10))
        self.T_map_marker = {}
        for m in db["markers"]:
            T = np.eye(4)
            T[:3, 3] = [m["x"], m["y"], m["z"]]
            T[:3, :3] = R.from_euler(
                "xyz", [m["roll"], m["pitch"], m["yaw"]]
            ).as_matrix()
            self.T_map_marker[int(m["id"])] = T

        if not hasattr(cv2.aruco, dict_name):
            self.get_logger().fatal(f"Diccionario desconocido: {dict_name}")
            sys.exit(1)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.aruco_params = (
            cv2.aruco.DetectorParameters()
            if hasattr(cv2.aruco, "DetectorParameters")
            else cv2.aruco.DetectorParameters_create()
        )
        # Refinamiento subpixel: mejora notablemente la precision del solvePnP
        if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.K = None
        self.D = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Filtro temporal
        win = int(self.get_parameter("filter_window").get_parameter_value().integer_value)
        self.pose_filter = PoseFilter(win)

        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").get_parameter_value().string_value,
            self.cam_info_cb, 10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").get_parameter_value().string_value,
            self.image_cb, qos_profile_sensor_data,
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/aruco_pose", 10
        )
        self.reproj_pub = self.create_publisher(Float32, "/aruco_pose/mean_reproj_error", 10)

        self.get_logger().info(
            f"Listo. dict={dict_name} | size={self.marker_size} m | "
            f"markers={len(self.T_map_marker)} | "
            f"max_dist={self.get_parameter('max_distance').value} m | "
            f"max_reproj={self.get_parameter('max_reproj_error_px').value} px | "
            f"filter_window={win}"
        )

    # -------------------- callbacks --------------------

    def cam_info_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    def image_cb(self, msg: Image):
        if self.K is None:
            return

        try:
            cv_img = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"conversion de imagen fallo: {e}")
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect_markers(gray, self.aruco_dict, self.aruco_params)
        if ids is None or len(ids) == 0:
            return

        camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

        try:
            tf_cam_to_base = self.tf_buffer.lookup_transform(
                camera_frame, base_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF {camera_frame}->{base_frame} no disponible: {e}",
                throttle_duration_sec=2.0,
            )
            return

        T_cam_base = to_matrix(
            [tf_cam_to_base.transform.translation.x,
             tf_cam_to_base.transform.translation.y,
             tf_cam_to_base.transform.translation.z],
            [tf_cam_to_base.transform.rotation.x,
             tf_cam_to_base.transform.rotation.y,
             tf_cam_to_base.transform.rotation.z,
             tf_cam_to_base.transform.rotation.w],
        )

        max_dist = float(self.get_parameter("max_distance").value)
        max_err = float(self.get_parameter("max_reproj_error_px").value)
        min_area = float(self.get_parameter("min_marker_area_px").value)
        amb_ratio = float(self.get_parameter("ambiguity_ratio_threshold").value)

        estimates = []
        diagnostics = []

        for i, marker_id in enumerate(ids.flatten().tolist()):
            if marker_id not in self.T_map_marker:
                continue

            area = marker_area_px(corners[i])
            if area < min_area:
                diagnostics.append(f"id={marker_id} REJ(area<{min_area:.0f})")
                continue

            sols = estimate_marker_pose_dual(corners[i], self.marker_size, self.K, self.D)
            if not sols:
                continue

            best = sols[0]
            rvec, tvec, err = best

            distance = float(np.linalg.norm(tvec))
            if max_dist > 0 and distance > max_dist:
                diagnostics.append(f"id={marker_id} REJ(d={distance:.2f}>{max_dist})")
                continue

            if err > max_err:
                diagnostics.append(f"id={marker_id} REJ(err={err:.2f}>{max_err}px)")
                continue

            # Aviso si la 2da solucion tiene error similar a la 1ra (pose ambigua)
            if len(sols) > 1:
                err2 = sols[1][2]
                if err2 < err * amb_ratio:
                    self.get_logger().warn(
                        f"id={marker_id} pose ambigua "
                        f"(err1={err:.2f} err2={err2:.2f} px). "
                        f"Recomendacion: aumentar inclinacion o tamano del marcador.",
                        throttle_duration_sec=3.0,
                    )

            T_marker_fix = np.eye(4)
            T_marker_fix[:3, :3] = R.from_euler('xyz', [0, 0, np.pi/2]).as_matrix()  # PRUEBA 1

            T_cam_marker = np.eye(4)
            T_cam_marker[:3, :3] = R.from_rotvec(rvec.flatten()).as_matrix()
            T_cam_marker[:3, 3]  = tvec.flatten()
            T_cam_marker = T_cam_marker @ T_marker_fix   # <-- aplicar correccion

            T_map_base = (
                self.T_map_marker[marker_id]
                @ np.linalg.inv(T_cam_marker)
                @ T_cam_base
            )
            

            # Peso = area / (1 + error_reproj). Marcadores grandes y nitidos pesan mas.
            weight = area / (1.0 + err)
            estimates.append((marker_id, T_map_base, weight, err, distance))
            diagnostics.append(
                f"id={marker_id} OK(d={distance:.2f}m,err={err:.2f}px,w={weight:.0f})"
            )

        if not estimates:
            if diagnostics:
                self.get_logger().info(
                    "Sin estimaciones validas: " + " | ".join(diagnostics),
                    throttle_duration_sec=1.0,
                )
            return

        Ts = [e[1] for e in estimates]
        ws = [e[2] for e in estimates]
        T_combined = average_poses(Ts, ws)

        # Filtro temporal
        T_filtered = self.pose_filter.update(T_combined)

        mean_err = float(np.mean([e[3] for e in estimates]))
        min_dist = float(min(e[4] for e in estimates))

        # Covarianza dinamica (ver comentario en __init__): escala con el error
        # de reproyeccion y la distancia al marcador mas cercano; promediar n
        # marcadores reduce la varianza ~1/n.
        n = len(estimates)
        err_scale = 1.0 + float(self.get_parameter("cov_reproj_scale").value) * mean_err
        dist_scale = 1.0 + float(self.get_parameter("cov_dist_scale").value) * min_dist ** 2
        cov_xy = float(self.get_parameter("base_cov_xy").value) * err_scale * dist_scale / n
        cov_yaw = float(self.get_parameter("base_cov_yaw").value) * err_scale / n

        self._publish(T_filtered, msg.header.stamp, cov_xy, cov_yaw)

        self.reproj_pub.publish(Float32(data=mean_err))

        ids_str = ",".join(str(e[0]) for e in estimates)
        x, y = T_filtered[0, 3], T_filtered[1, 3]
        yaw = R.from_matrix(T_filtered[:3, :3]).as_euler("xyz")[2]
        self.get_logger().info(
            f"[ids={ids_str}] map=({x:+.3f}, {y:+.3f}, "
            f"yaw={np.degrees(yaw):+.1f} deg) reproj_avg={mean_err:.2f}px",
            throttle_duration_sec=0.5,
        )

    # -------------------- helpers --------------------

    def _publish(self, T_map_base, stamp, cov_xy=0.075, cov_yaw=0.15):
        t, q = matrix_to_translation_quat(T_map_base)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.get_parameter("map_frame").value
        msg.pose.pose.position.x = float(t[0])
        msg.pose.pose.position.y = float(t[1])
        msg.pose.pose.position.z = float(t[2])
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])
        cov = list(msg.pose.covariance)
        cov[0] = float(cov_xy)
        cov[7] = float(cov_xy)
        cov[35] = float(cov_yaw)
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)

        if not self.get_parameter("publish_tf").value:
            return

        odom_frame = self.get_parameter("odom_frame").value
        base_frame = self.get_parameter("base_frame").value
        try:
            tf_odom_base = self.tf_buffer.lookup_transform(odom_frame, base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"No pude leer {odom_frame}->{base_frame}: {e}",
                                   throttle_duration_sec=2.0)
            return
        T_odom_base = to_matrix(
            [tf_odom_base.transform.translation.x,
             tf_odom_base.transform.translation.y,
             tf_odom_base.transform.translation.z],
            [tf_odom_base.transform.rotation.x,
             tf_odom_base.transform.rotation.y,
             tf_odom_base.transform.rotation.z,
             tf_odom_base.transform.rotation.w],
        )
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
        t2, q2 = matrix_to_translation_quat(T_map_odom)

        out = TransformStamped()
        out.header.stamp = stamp
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = odom_frame
        out.transform.translation.x = float(t2[0])
        out.transform.translation.y = float(t2[1])
        out.transform.translation.z = float(t2[2])
        out.transform.rotation.x = float(q2[0])
        out.transform.rotation.y = float(q2[1])
        out.transform.rotation.z = float(q2[2])
        out.transform.rotation.w = float(q2[3])
        self.tf_broadcaster.sendTransform(out)


def main():
    rclpy.init()
    node = ArucoLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()