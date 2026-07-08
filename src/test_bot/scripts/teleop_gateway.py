#!/usr/bin/env python3
"""
teleop_gateway.py
-----------------
Gateway que reexpone a la GUI (capbot-host) los MISMOS protocolos que daba
capbot-jetson-bridge, para que la GUI funcione SIN cambios:

  - UDP 5005 (host -> jetson): comandos en frames de 16 bytes
        [magic:0xABCD][ver:1][type:1][seq:4][payload:6][crc16-ccitt]
    -> los traduce a topicos ROS que consume esp32_serial_bridge.
  - UDP 5006 (jetson -> host): ACK por cada comando (el host reintenta hasta el ACK).
  - WS 8765 (jetson -> host): difunde la telemetria JSON (de /esp32/telemetry).

MsgTypes UDP (capbot-host/protocol/udp_frame.py):
  0x01 CMD_MOTOR    <hhh> left,right,aux  -> /esp32/motor_cmd (manual, por rueda)
  0x02 CMD_HEARTBEAT                       -> keepalive (solo ACK)
  0x03 CMD_EMERGENCY                       -> /esp32/estop (True)
  0x04 CMD_PID_PARAM <BBf> ctrl,param,val  -> /esp32/pid_param  (modo PID)
  0x06 CMD_MODE      <B>  mode             -> /esp32/mode (0=manual,1=auto)
  0x81 ACK (lo enviamos nosotros)

El contenedor corre con network_mode host, asi que estos puertos son los de la
Jetson y la GUI los alcanza igual que antes.
"""
import socket
import struct
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int8, Int16MultiArray, Float32MultiArray, String

try:
    import asyncio
    import websockets
except ImportError:
    websockets = None


# --------------------------------------------------------------------------
# Protocolo UDP 16B (espejo de capbot-host/protocol/udp_frame.py)
# --------------------------------------------------------------------------
MAGIC = 0xABCD
VERSION = 1
FRAME_SIZE = 16

CMD_MOTOR = 0x01
CMD_HEARTBEAT = 0x02
CMD_EMERGENCY = 0x03
CMD_PID_PARAM = 0x04
CMD_MODE = 0x06
ACK = 0x81


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


def parse_frame(data):
    """Devuelve (type, seq, payload6) o None si invalido."""
    if len(data) != FRAME_SIZE:
        return None
    magic, version, mtype, seq, payload, crc = struct.unpack("<HBBI6sH", data)
    if magic != MAGIC or version != VERSION:
        return None
    if crc16_ccitt(data[:14]) != crc:
        return None
    return mtype, seq, payload


def build_ack(seq):
    payload = struct.pack("<I", seq & 0xFFFFFFFF) + b"\x00\x00"
    header = struct.pack("<HBBI6s", MAGIC, VERSION, ACK, seq & 0xFFFFFFFF, payload)
    return header + struct.pack("<H", crc16_ccitt(header))


class TeleopGateway(Node):
    def __init__(self):
        super().__init__("teleop_gateway")

        self.declare_parameter("udp_cmd_port", 5005)
        self.declare_parameter("udp_ack_port", 5006)
        self.declare_parameter("ws_telemetry_port", 8765)
        gp = self.get_parameter
        self.cmd_port = int(gp("udp_cmd_port").value)
        self.ack_port = int(gp("udp_ack_port").value)
        self.ws_port = int(gp("ws_telemetry_port").value)

        # Publishers hacia esp32_serial_bridge
        self.motor_pub = self.create_publisher(Int16MultiArray, "/esp32/motor_cmd", 10)
        self.estop_pub = self.create_publisher(Bool, "/esp32/estop", 10)
        self.pid_pub = self.create_publisher(Float32MultiArray, "/esp32/pid_param", 10)
        self.mode_pub = self.create_publisher(Int8, "/esp32/mode", 10)

        # Telemetria -> WS
        self._latest_telem = None
        self._telem_lock = threading.Lock()
        self.create_subscription(String, "/esp32/telemetry", self._on_telem, 10)

        self._running = True
        self._udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self._udp_thread.start()

        if websockets is not None:
            self._ws_thread = threading.Thread(target=self._ws_loop, daemon=True)
            self._ws_thread.start()
        else:
            self.get_logger().warn("websockets no instalado; WS de telemetria deshabilitado.")

        self.get_logger().info(
            "teleop_gateway: UDP cmd %d (+ACK %d) -> /esp32/*, WS telemetria %d."
            % (self.cmd_port, self.ack_port, self.ws_port))

    # ----------------------- telemetria -----------------------
    def _on_telem(self, msg):
        with self._telem_lock:
            self._latest_telem = msg.data

    # ----------------------- UDP comandos -----------------------
    def _udp_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.cmd_port))
        sock.settimeout(0.5)
        ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while self._running:
            try:
                data, addr = sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            fr = parse_frame(data)
            if fr is None:
                continue
            mtype, seq, payload = fr
            self._dispatch(mtype, payload)
            # ACK al host (auto-deteccion de IP desde el remitente).
            try:
                ack_sock.sendto(build_ack(seq), (addr[0], self.ack_port))
            except OSError:
                pass

    def _dispatch(self, mtype, payload):
        if mtype == CMD_MOTOR:
            left, right, _aux = struct.unpack("<hhh", payload)
            m = Int16MultiArray()
            m.data = [int(left), int(right)]
            self.motor_pub.publish(m)
        elif mtype == CMD_EMERGENCY:
            self.estop_pub.publish(Bool(data=True))
        elif mtype == CMD_PID_PARAM:
            ctrl, param, val = struct.unpack("<BBf", payload)
            f = Float32MultiArray()
            f.data = [float(ctrl), float(param), float(val)]
            self.pid_pub.publish(f)
        elif mtype == CMD_MODE:
            self.mode_pub.publish(Int8(data=int(payload[0])))
        elif mtype == CMD_HEARTBEAT:
            pass  # keepalive; el ACK basta

    # ----------------------- WS telemetria -----------------------
    def _ws_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        async def handler(ws, path=None):
            try:
                while self._running:
                    with self._telem_lock:
                        cur = self._latest_telem
                    # Mandar siempre (no solo cuando cambia): con el robot
                    # quieto el JSON de telemetria puede quedar identico
                    # frame a frame (v/w en 0), y el host marca la
                    # telemetria como obsoleta si no le llega nada nuevo.
                    if cur is not None:
                        await ws.send(cur)
                    await asyncio.sleep(0.05)   # 20 Hz
            except Exception:
                pass

        try:
            server = websockets.serve(handler, "0.0.0.0", self.ws_port)
            loop.run_until_complete(server)
            loop.run_forever()
        except Exception as e:
            self.get_logger().error("WS telemetria fallo: %s" % e)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main():
    rclpy.init()
    node = TeleopGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
