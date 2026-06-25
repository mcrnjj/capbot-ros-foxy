#!/usr/bin/env python3
"""
esp32_serial_bridge.py
----------------------
Puente serie (COBS+CRC16) entre la pila ROS2 y el firmware del ESP32
(capbot-ESP32). El contenedor es DUENO del puerto /dev/ttyTHS1 (Opcion B).

Protocolo (byte-compatible con capbot-jetson-bridge/protocol/cobs_frame.py):
  frame en el cable:  COBS( [type:1][len:1][payload:len][crc16:2] ) + 0x00
  CRC16-CCITT poly=0x1021 init=0xFFFF sobre [type+len+payload].

MsgTypes (capbot-ESP32/include/Config.h):
  0x10 MOTOR_CMD   <hhh> (left,right,aux)   PWM crudo por rueda (modo MANUAL)
  0x11 BRAKE_ON    -                         freno activo
  0x12 HEARTBEAT   -                         para el watchdog del ESP32 (<200ms)
  0x14 SETPOINT_COMP <BBf> (comp,res,val)    setpoint de posicion (modo AUTO)
  0x15 MODE_CMD    <B> (0=manual,1=auto)
  0x16 VEL_CMD     <ff> (v[m/s], w[deg/s])   *** NUEVO; firmware Fase 4 ***
  0x20 TELEMETRY   JSON UTF-8 (ESP32->Jetson)
  0x21 ESP_HELLO   -

Funciones:
  - Lee TELEMETRY (JSON {mode,u,odo:{x,y,a,v,w},sp,error}) -> nav_msgs/Odometry
    en /odom. odo.a (theta) y odo.w (omega) vienen en GRADOS/grados-s -> rad.
  - /cmd_vel (Twist) -> MODE_CMD(1) + VEL_CMD(v m/s, w deg/s).
    OJO: el firmware actual aun NO maneja VEL_CMD (Fase 4); lo descarta de forma
    segura (CRC valida, tipo desconocido ignorado). Inerte hasta el cambio.
  - /esp32/motor_cmd (Int16MultiArray [left,right]) -> MODE_CMD(0) + MOTOR_CMD
    (teleop manual; lo alimenta teleop_gateway/cmd_mux).
  - /esp32/estop (Bool True) -> BRAKE_ON.
  - HEARTBEAT cada 50ms; BRAKE al cerrar; watchdog de /cmd_vel (frena si se corta).

El EKF publica el TF odom->base_link, asi que aca publish_odom_tf=false por defecto.
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int8, Int16MultiArray, Float32MultiArray, String
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None


# --------------------------------------------------------------------------
# Protocolo COBS + CRC16 (espejo de capbot-jetson-bridge/protocol/cobs_frame.py)
# --------------------------------------------------------------------------
DELIMITER = 0x00

MOTOR_CMD = 0x10
BRAKE_ON = 0x11
HEARTBEAT = 0x12
PID_PARAM = 0x13
SETPOINT_COMP = 0x14
MODE_CMD = 0x15
VEL_CMD = 0x16
TELEMETRY = 0x20
ESP_HELLO = 0x21


def crc16_ccitt(data, init=0xFFFF):
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data):
    out = bytearray([0])
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data):
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise ValueError("cero inesperado en COBS")
        end = i + code
        if end > n:
            raise ValueError("codigo COBS se pasa del final")
        out.extend(data[i + 1:end])
        i = end
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def pack_frame(msg_type, payload=b""):
    raw = struct.pack("<BB", msg_type & 0xFF, len(payload)) + payload
    raw += struct.pack("<H", crc16_ccitt(raw))
    return cobs_encode(raw) + bytes([DELIMITER])


def unpack_frame(encoded):
    raw = cobs_decode(encoded)
    if len(raw) < 4:
        raise ValueError("frame truncado")
    msg_type, length = raw[0], raw[1]
    if len(raw) != 2 + length + 2:
        raise ValueError("longitud inconsistente")
    payload = raw[2:2 + length]
    (crc_recv,) = struct.unpack("<H", raw[2 + length:])
    if crc_recv != crc16_ccitt(raw[:2 + length]):
        raise ValueError("CRC invalido")
    return msg_type, payload


class FrameBuffer:
    """Acumula bytes y entrega (type, payload) por cada 0x00."""

    def __init__(self, max_bytes=512):
        self._buf = bytearray()
        self._max = max_bytes

    def feed(self, data):
        frames = []
        for b in data:
            if b == DELIMITER:
                if self._buf:
                    try:
                        frames.append(unpack_frame(bytes(self._buf)))
                    except ValueError:
                        pass
                    self._buf.clear()
            else:
                self._buf.append(b)
                if len(self._buf) > self._max:
                    self._buf.clear()
        return frames


# --------------------------------------------------------------------------
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Esp32SerialBridge(Node):
    def __init__(self):
        super().__init__("esp32_serial_bridge")

        self.declare_parameter("serial_port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_odom_tf", False)  # el EKF da odom->base_link
        self.declare_parameter("max_linear_speed", 0.3)   # m/s, clamp /cmd_vel
        self.declare_parameter("max_angular_speed", 2.0)  # rad/s, clamp /cmd_vel
        self.declare_parameter("cmd_vel_timeout", 0.5)    # s, frena si se corta
        self.declare_parameter("heartbeat_period", 0.05)  # s (<200ms watchdog FW)

        gp = self.get_parameter
        self.port = gp("serial_port").value
        self.baud = gp("baudrate").value
        self.odom_frame = gp("odom_frame").value
        self.base_frame = gp("base_frame").value
        self.publish_tf = bool(gp("publish_odom_tf").value)
        self.max_lin = float(gp("max_linear_speed").value)
        self.max_ang = float(gp("max_angular_speed").value)
        self.cmd_vel_timeout = float(gp("cmd_vel_timeout").value)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.telem_pub = self.create_publisher(String, "/esp32/telemetry", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Int16MultiArray, "/esp32/motor_cmd", self._on_motor_cmd, 10)
        self.create_subscription(Bool, "/esp32/estop", self._on_estop, 10)
        # PID (0x13), setpoint (0x14) y modo (0x15): los alimenta teleop_gateway
        # desde la GUI (modo PID / control). data como Float32MultiArray.
        self.create_subscription(Float32MultiArray, "/esp32/pid_param", self._on_pid_param, 10)
        self.create_subscription(Float32MultiArray, "/esp32/setpoint", self._on_setpoint, 10)
        self.create_subscription(Int8, "/esp32/mode", self._on_mode_cmd, 10)

        # Estado
        self._mode = None            # None/0/1 para no spamear MODE_CMD
        self._last_cmd_vel_t = 0.0
        self._cmd_vel_active = False
        self._ser = None
        self._write_lock = threading.Lock()

        if serial is None:
            self.get_logger().fatal("pyserial no instalado (pip install pyserial).")
            raise SystemExit(1)
        self._open_serial()

        # Lector serie en hilo daemon
        self._buffer = FrameBuffer()
        self._running = True
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

        # Heartbeat + watchdog en timers ROS (corren en el hilo de spin)
        hb = float(gp("heartbeat_period").value)
        self.create_timer(hb, self._send_heartbeat)
        self.create_timer(0.1, self._check_cmd_vel_timeout)

        self.get_logger().info(
            "esp32_serial_bridge listo en %s @ %d. /cmd_vel->VEL_CMD (v m/s, w deg/s; "
            "INERTE hasta firmware Fase 4), /esp32/motor_cmd->MOTOR_CMD (manual)."
            % (self.port, self.baud))

    # ----------------------- serie -----------------------
    def _open_serial(self):
        try:
            self._ser = serial.Serial(
                port=self.port, baudrate=self.baud,
                timeout=0.05, write_timeout=0.2, rtscts=False, dsrdtr=False)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self.get_logger().info("Serie ESP32 abierto: %s" % self.port)
        except Exception as e:
            self.get_logger().fatal("No pude abrir %s: %s" % (self.port, e))
            raise SystemExit(1)

    def _write(self, frame):
        if self._ser is None:
            return
        with self._write_lock:
            try:
                self._ser.write(frame)
            except Exception as e:
                self.get_logger().warn("Error escribiendo serie: %s" % e,
                                       throttle_duration_sec=2.0)

    def _reader_loop(self):
        while self._running:
            try:
                n = self._ser.in_waiting
                data = self._ser.read(n if n > 0 else 1)
            except Exception as e:
                self.get_logger().warn("Error leyendo serie: %s" % e,
                                       throttle_duration_sec=2.0)
                # Backoff: sin esto, ante error de I/O (ESP32 desconectado) el
                # read() falla al instante y el loop gira al 100% de CPU,
                # degradando el resto (camara/GStreamer).
                time.sleep(0.5)
                continue
            if not data:
                continue
            for msg_type, payload in self._buffer.feed(data):
                if msg_type == TELEMETRY:
                    self._handle_telemetry(payload)
                elif msg_type == ESP_HELLO:
                    self.get_logger().info("ESP32 HELLO")

    # ----------------------- telemetria -> /odom -----------------------
    def _handle_telemetry(self, payload):
        import json
        try:
            text = payload.decode("utf-8")
            data = json.loads(text)
        except (UnicodeDecodeError, ValueError):
            return
        # Reenvia la telemetria cruda (JSON) -> teleop_gateway la difunde por WS a la GUI.
        smsg = String()
        smsg.data = text
        self.telem_pub.publish(smsg)
        odo = data.get("odo")
        if not isinstance(odo, dict):
            return
        try:
            x = float(odo["x"]); y = float(odo["y"])
            theta = float(odo["a"]) * DEG2RAD     # grados -> rad
            v = float(odo["v"])
            w = float(odo["w"]) * DEG2RAD          # grados/s -> rad/s
        except (KeyError, TypeError, ValueError):
            return

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(theta)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        # Covarianzas (el EKF usa sobre todo el twist; la pose la corrige ArUco).
        msg.twist.covariance[0] = 0.02     # vx
        msg.twist.covariance[35] = 0.04    # vyaw
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.10
        self.odom_pub.publish(msg)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

    # ----------------------- comandos -----------------------
    def _set_mode(self, mode):
        if self._mode != mode:
            self._mode = mode
            self._write(pack_frame(MODE_CMD, struct.pack("<B", mode & 0xFF)))

    def _on_cmd_vel(self, msg):
        self._last_cmd_vel_t = self.get_clock().now().nanoseconds * 1e-9
        self._cmd_vel_active = True
        self._set_mode(1)  # autonomo
        v = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        w_rad = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        w_deg = w_rad * RAD2DEG           # firmware espera deg/s
        self._write(pack_frame(VEL_CMD, struct.pack("<ff", v, w_deg)))

    def _on_motor_cmd(self, msg):
        if len(msg.data) < 2:
            return
        self._set_mode(0)  # manual
        left = int(max(-32768, min(32767, msg.data[0])))
        right = int(max(-32768, min(32767, msg.data[1])))
        self._write(pack_frame(MOTOR_CMD, struct.pack("<hhh", left, right, 0)))

    def _on_estop(self, msg):
        if msg.data:
            self._write(pack_frame(BRAKE_ON))
            self._cmd_vel_active = False

    def _on_pid_param(self, msg):
        # [ctrl_id, param_id, value] -> PID_PARAM 0x13 <BBf> (igual que cobs_frame.py).
        if len(msg.data) < 3:
            return
        ctrl = int(msg.data[0]) & 0xFF
        param = int(msg.data[1]) & 0xFF
        value = float(msg.data[2])
        self._write(pack_frame(PID_PARAM, struct.pack("<BBf", ctrl, param, value)))

    def _on_setpoint(self, msg):
        # [comp_id, value] -> SETPOINT_COMP 0x14 <BBf>.
        if len(msg.data) < 2:
            return
        comp = int(msg.data[0]) & 0xFF
        value = float(msg.data[1])
        self._write(pack_frame(SETPOINT_COMP, struct.pack("<BBf", comp, 0, value)))

    def _on_mode_cmd(self, msg):
        self._set_mode(int(msg.data) & 0x01)

    def _check_cmd_vel_timeout(self):
        if not self._cmd_vel_active:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._last_cmd_vel_t) > self.cmd_vel_timeout:
            self._cmd_vel_active = False
            self._write(pack_frame(VEL_CMD, struct.pack("<ff", 0.0, 0.0)))
            self.get_logger().warn("Sin /cmd_vel; frenando (VEL_CMD 0,0).")

    def _send_heartbeat(self):
        self._write(pack_frame(HEARTBEAT))

    def destroy_node(self):
        self._running = False
        try:
            self._write(pack_frame(BRAKE_ON))
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Esp32SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
