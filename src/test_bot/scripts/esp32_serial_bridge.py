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
  0x10 MOTOR_CMD      <hhh> (left,right,aux)   PWM crudo por rueda (modo MANUAL)
  0x11 BRAKE_ON       -                         freno activo
  0x12 HEARTBEAT      -                         para el watchdog del ESP32 (<200ms)
  0x14 SETPOINT_COMP  <BBf> (comp,res,val)      setpoint de posicion (modo WAYPOINT)
  0x15 MODE_CMD       <B> (0=manual,1=autonomous_nav,2=autonomous_waypoint)
  0x16 WHEEL_VEL_CMD  <ff> (wheelLeft, wheelRight) [rad/s] modo AUTONOMOUS_NAV:
       setpoints de velocidad POR RUEDA. El firmware ya no mezcla lineal/
       angular ni corre odometria propia: cada rueda tiene su PID
       independiente (Controlador::leftWheelPid/rightWheelPid) contra este
       setpoint en rad/s. Este puente hace el mixing diferencial de /cmd_vel
       (Twist) -> (wheelLeft, wheelRight) (ver _on_cmd_vel).
  0x20 TELEMETRY      JSON UTF-8 (ESP32->Jetson), cada TELEMETRY_PERIOD_MS (20ms).
       Espejo de SensorHub::buildPayload (capbot-ESP32):
       {mode: "manual"|...,
        u: {enc_left, enc_right,           cuentas de encoder acumuladas (crudo)
            vel_left_cps, vel_right_cps,   cuentas/s crudas (NO son rad/s ni m/s)
            pwm_left, pwm_right, braking}, estado del driver de motores
        ctrl: {sp_left, sp_right}}         setpoint activo por rueda (rad/s)
       Ya NO trae pose (x,y,theta,v,w): la clase Odometry que fusionaba
       encoders+IMU se elimino del firmware. "la navegacion/pose vive en
       nav2 + EKF, en la Jetson" (comentario en ESP32/main.cpp) -> este
       puente calcula la odometria (velocidad de rueda -> v,w -> pose 2D)
       a partir de vel_left_cps/vel_right_cps y las constantes geometricas
       del robot (wheel_radius, wheel_separation, wheel_cpr).
       El puente ademas AGREGA un campo nav_ref (no viene del ESP32) antes
       de reenviar por /esp32/telemetry y WS:
       {..., nav_ref: {v, w,               Twist crudo (clamped) de /cmd_vel
                       wheel_left, wheel_right}}  mismo mixing que WHEEL_VEL_CMD
       Es la referencia que el Jetson esta comandando (nav2), no lo que el
       firmware esta logrando (eso sigue en ctrl.sp_left/sp_right).
  0x21 ESP_HELLO      -

Funciones:
  - Lee TELEMETRY (JSON crudo de encoders/PWM/setpoints) -> calcula odometria
    diferencial en el Jetson -> publica nav_msgs/Odometry en /odom.
  - /cmd_vel (Twist) -> MODE_CMD(1) + WHEEL_VEL_CMD (wheelLeft, wheelRight
    en rad/s, mixing diferencial estandar con wheel_radius/wheel_separation).
  - /esp32/motor_cmd (Int16MultiArray [left,right]) -> MODE_CMD(0) + MOTOR_CMD
    (teleop manual; lo alimenta teleop_gateway).
  - /esp32/estop (Bool True) -> BRAKE_ON.
  - HEARTBEAT cada 50ms; BRAKE al cerrar.
  - Lazo de control a tasa fija (control_period, 20 Hz): los callbacks solo
    actualizan un buffer con el ultimo comando; _control_tick lo reenvia
    continuamente al ESP32 para que el setpoint del firmware nunca envejezca
    entre ciclos del planner (nav2 publica a ~5 Hz vs NAV_VEL_TIMEOUT_MS=300ms).
    Si la fuente activa se corta (cmd_vel_timeout) manda UN stop y deja de enviar.
  - Arbitraje de modo "sticky": la fuente que tiene el modo lo conserva mientras
    sus comandos esten frescos; la otra fuente no puede robarlo. Un /esp32/mode
    explicito (boton GUI o goal de navegacion via gui_bridge_node) cambia el modo
    al instante con una ventana de gracia (mode_switch_grace).

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
WHEEL_VEL_CMD = 0x16
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
TWO_PI = 2.0 * math.pi


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
        self.declare_parameter("cmd_vel_timeout", 0.5)    # s, frena si el cmd activo se corta
        self.declare_parameter("control_period", 0.05)    # s, resend continuo del ultimo cmd
        self.declare_parameter("mode_switch_grace", 2.0)  # s, ventana tras /esp32/mode explicito
        self.declare_parameter("heartbeat_period", 0.05)  # s (<200ms watchdog FW)
        # Geometria del robot (debe calzar con description/robot_core.xacro) y
        # constante de encoder del firmware (Cfg::WHEEL_CPR en Config.h):
        # cuentas por vuelta de rueda en cuadratura 4x. Con esto se convierte
        # vel_*_cps (cuentas/s crudas) <-> rad/s de cada rueda.
        self.declare_parameter("wheel_radius", 0.035)      # m
        self.declare_parameter("wheel_separation", 0.17)   # m (track width)
        self.declare_parameter("wheel_cpr", 898)            # cuentas/vuelta (4x)

        gp = self.get_parameter
        self.port = gp("serial_port").value
        self.baud = gp("baudrate").value
        self.odom_frame = gp("odom_frame").value
        self.base_frame = gp("base_frame").value
        self.publish_tf = bool(gp("publish_odom_tf").value)
        self.max_lin = float(gp("max_linear_speed").value)
        self.max_ang = float(gp("max_angular_speed").value)
        self.cmd_vel_timeout = float(gp("cmd_vel_timeout").value)
        self.control_period = float(gp("control_period").value)
        self.mode_switch_grace = float(gp("mode_switch_grace").value)
        self.wheel_radius = float(gp("wheel_radius").value)
        self.wheel_separation = float(gp("wheel_separation").value)
        self.wheel_cpr = float(gp("wheel_cpr").value)

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
        # Buffer/timeout compartido por AMBAS fuentes (manual y nav): solo una
        # esta activa a la vez (la que fijo el _mode actual), asi que basta un
        # unico timestamp/flag de vigencia en vez de duplicar el mecanismo.
        # Arbitraje "sticky": mientras la fuente activa este fresca, la otra
        # NO puede robarle el modo. Un /esp32/mode explicito si cambia el modo
        # al instante y abre una ventana de gracia (_hold_until) para que la
        # nueva fuente alcance a publicar sin perder el control.
        self._last_cmd_t = None      # timestamp (s) del ultimo cmd de la fuente activa
        self._cmd_active = False
        self._hold_until = 0.0       # hasta este t (s) el modo actual es inrobable
        self._last_wheel_cmd = (0.0, 0.0)   # (rad/s izq, rad/s der) — buffer modo nav
        self._last_cmd_vw = (0.0, 0.0)      # (v m/s, w rad/s) — Twist crudo de /cmd_vel (nav2)
        self._last_motor_cmd = (0, 0)       # (left, right) PWM crudo — buffer modo manual
        self._ser = None
        self._write_lock = threading.Lock()

        # Odometria (integrada aca; el ESP32 solo manda encoders/velocidades crudas).
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_telem_t = None    # timestamp (s) de la ultima TELEMETRY

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
        self.create_timer(self.control_period, self._control_tick)

        self.get_logger().info(
            "esp32_serial_bridge listo en %s @ %d. /cmd_vel->WHEEL_VEL_CMD "
            "(rad/s por rueda), /esp32/motor_cmd->MOTOR_CMD (manual). "
            "Odometria calculada en el Jetson desde vel_*_cps."
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

    # ----------------------- telemetria -> odometria -> /odom -----------------------
    def _handle_telemetry(self, payload):
        import json
        try:
            text = payload.decode("utf-8")
            data = json.loads(text)
        except (UnicodeDecodeError, ValueError):
            return
        # Agrega la referencia de velocidad de nav2 (lo que el Jetson esta
        # comandando, no lo que el firmware esta logrando) antes de reenviar
        # la telemetria -> teleop_gateway la difunde por WS a la GUI.
        v, w = self._last_cmd_vw
        wl_ref, wr_ref = self._last_wheel_cmd
        data["nav_ref"] = {"v": v, "w": w, "wheel_left": wl_ref, "wheel_right": wr_ref}
        smsg = String()
        smsg.data = json.dumps(data)
        self.telem_pub.publish(smsg)

        try:
            vel_left_cps = float(data["u"]["vel_left_cps"])
            vel_right_cps = float(data["u"]["vel_right_cps"])
        except (KeyError, TypeError, ValueError):
            return

        # cuentas/s -> rad/s de cada rueda -> m/s tangencial de cada rueda.
        omega_left = (vel_left_cps / self.wheel_cpr) * TWO_PI
        omega_right = (vel_right_cps / self.wheel_cpr) * TWO_PI
        v_left = omega_left * self.wheel_radius
        v_right = omega_right * self.wheel_radius

        # Cinematica diferencial estandar.
        v = (v_left + v_right) / 2.0
        w = (v_right - v_left) / self.wheel_separation

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_telem_t is None or (now - self._last_telem_t) > 0.5:
            dt = 0.0
        else:
            dt = now - self._last_telem_t
        self._last_telem_t = now

        # Integracion "midpoint" (igual que diff_drive_controller): mas precisa
        # que Euler simple cuando w != 0 dentro del intervalo.
        half_dtheta = 0.5 * w * dt
        self._x += v * math.cos(self._theta + half_dtheta) * dt
        self._y += v * math.sin(self._theta + half_dtheta) * dt
        self._theta += w * dt
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))  # wrap +-pi

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(self._theta)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
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
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
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

    def _wheel_speeds(self, v, w):
        """Mixing diferencial: (v,w) del chasis -> (rad/s izq, rad/s der)."""
        v_left = v - w * (self.wheel_separation / 2.0)
        v_right = v + w * (self.wheel_separation / 2.0)
        return v_left / self.wheel_radius, v_right / self.wheel_radius

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _mode_locked(self, now):
        """True si el modo actual NO puede ser robado por la otra fuente."""
        if now < self._hold_until:
            return True
        return (self._cmd_active and self._last_cmd_t is not None
                and (now - self._last_cmd_t) <= self.cmd_vel_timeout)

    def _on_cmd_vel(self, msg):
        """Solo actualiza el buffer; el envio real lo hace _control_tick a tasa fija."""
        now = self._now()
        if self._mode == 0 and self._mode_locked(now):
            return  # manual tiene el control y esta fresco: nav no lo roba
        v = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        w = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        self._last_cmd_vw = (v, w)
        self._last_wheel_cmd = self._wheel_speeds(v, w)
        self._last_cmd_t = now
        self._cmd_active = True
        self._set_mode(1)  # autonomous_nav

    def _on_motor_cmd(self, msg):
        """Solo actualiza el buffer; el envio real lo hace _control_tick a tasa fija."""
        if len(msg.data) < 2:
            return
        now = self._now()
        if self._mode == 1 and self._mode_locked(now):
            return  # nav tiene el control y esta fresco: manual no lo roba
        left = int(max(-32768, min(32767, msg.data[0])))
        right = int(max(-32768, min(32767, msg.data[1])))
        self._last_motor_cmd = (left, right)
        self._last_cmd_t = now
        self._cmd_active = True
        self._set_mode(0)  # manual

    def _on_estop(self, msg):
        if msg.data:
            self._write(pack_frame(BRAKE_ON))
            self._cmd_active = False
            self._hold_until = 0.0
            self._last_wheel_cmd = (0.0, 0.0)
            self._last_cmd_vw = (0.0, 0.0)
            self._last_motor_cmd = (0, 0)

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
        """Cambio de modo EXPLICITO (boton de la GUI o goal de navegacion).

        Siempre gana sobre el arbitraje implicito: fija el modo, limpia los
        buffers (arranca detenido) y abre una ventana de gracia para que la
        nueva fuente alcance a publicar antes de que la otra pueda reclamar.
        """
        mode = int(msg.data) & 0x01
        if mode == self._mode:
            return
        self._set_mode(mode)
        self._last_wheel_cmd = (0.0, 0.0)
        self._last_cmd_vw = (0.0, 0.0)
        self._last_motor_cmd = (0, 0)
        self._cmd_active = True
        self._last_cmd_t = self._now()
        self._hold_until = self._last_cmd_t + self.mode_switch_grace

    def _control_tick(self):
        """Reenvia el ultimo comando bufferizado a tasa fija (control_period).

        Esto desacopla la tasa de envio al ESP32 de la tasa a la que llegan
        los comandos ROS: nav2 publica /cmd_vel a ~5 Hz, muy cerca del
        NAV_VEL_TIMEOUT_MS=300ms del firmware, y cualquier jitter hacia que
        el firmware frenara entre ciclos del planner (movimiento a tirones).
        Con este resend continuo el setpoint del firmware nunca envejece
        mientras la fuente activa siga viva.
        """
        if not self._cmd_active:
            return
        now = self._now()
        if not self._mode_locked(now):
            # La fuente activa se corto: un solo stop y dejar de reenviar.
            self._cmd_active = False
            self._last_wheel_cmd = (0.0, 0.0)
            self._last_motor_cmd = (0, 0)
            if self._mode == 1:
                self._write(pack_frame(WHEEL_VEL_CMD, struct.pack("<ff", 0.0, 0.0)))
            else:
                self._write(pack_frame(MOTOR_CMD, struct.pack("<hhh", 0, 0, 0)))
            self.get_logger().warn("Fuente de comandos cortada; frenando.")
            return
        if self._mode == 1:
            wl, wr = self._last_wheel_cmd
            self._write(pack_frame(WHEEL_VEL_CMD, struct.pack("<ff", wl, wr)))
        elif self._mode == 0:
            left, right = self._last_motor_cmd
            self._write(pack_frame(MOTOR_CMD, struct.pack("<hhh", left, right, 0)))

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
