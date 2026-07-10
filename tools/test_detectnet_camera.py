#!/usr/bin/env python3
"""
test_detectnet_camera.py
------------------------
Prueba AISLADA de jetson-inference (sin ROS2): abre la camara CSI via
jetson_utils.videoSource, corre detectNet en GPU, GRABA un video .mp4 con los
overlays (NVENC) y deja un log CSV de detecciones + resumen de FPS.

Que valida:
  - captura CSI (Argus) dentro del contenedor
  - engine TensorRT + modelo (steady-state FPS real, no el warm-up)
  - encoder por hardware (misma carga que la rama de video del robot)
  - deteccion de TUS obstaculos reales (revisar el video/CSV despues)

Que NO usa: rclpy, topics, TF. Si esto funciona y el nodo ROS no, el problema
esta en la integracion ROS, no en camara/GPU.

IMPORTANTE: cerrar el stack ROS antes (Argus es single-client: si
csi_camera_node tiene la camara, aqui fallara la captura).

Uso tipico (ver README del test al final del archivo):
  python3 test_detectnet_camera.py --duration 20
  python3 test_detectnet_camera.py --duration 30 --no-video   # sin NVENC (menos consumo)
"""

import argparse
import csv
import os
import signal
import sys
import time

# API nueva (jetson_inference) o legacy (jetson.inference)
try:
    from jetson_inference import detectNet
    from jetson_utils import videoSource, videoOutput
except ImportError:
    from jetson.inference import detectNet
    from jetson.utils import videoSource, videoOutput


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--network", default="ssd-mobilenet-v2")
    p.add_argument("--threshold", type=float, default=0.5)
    p.add_argument("--camera", default="csi://0")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=15,
                   help="framerate de captura (15 = menos consumo)")
    p.add_argument("--duration", type=float, default=20.0,
                   help="segundos de prueba (0 = hasta Ctrl+C)")
    p.add_argument("--out-dir", default="/test/out",
                   help="directorio de salida (montar en el host)")
    p.add_argument("--bitrate", type=int, default=2000000,
                   help="bitrate NVENC en bps para el mp4")
    p.add_argument("--no-video", action="store_true",
                   help="no grabar video (sin NVENC): solo stats + CSV")
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    video_path = os.path.join(args.out_dir, "detect_%s.mp4" % stamp)
    csv_path = os.path.join(args.out_dir, "detect_%s.csv" % stamp)

    print("[test] cargando red '%s' (el 1er frame paga warm-up ~1.5 s)..." % args.network)
    net = detectNet(args.network, threshold=args.threshold)

    cam = videoSource(args.camera, argv=[
        "--input-width=%d" % args.width,
        "--input-height=%d" % args.height,
        "--input-rate=%d" % args.fps,
    ])

    out = None
    if not args.no_video:
        out = videoOutput("file://%s" % video_path, argv=[
            "--headless",
            "--bitrate=%d" % args.bitrate,
        ])
        print("[test] grabando en %s (NVENC %d kbps)" % (video_path, args.bitrate // 1000))
    else:
        print("[test] --no-video: sin encoder (prueba de menor consumo)")

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))

    csv_f = open(csv_path, "w", newline="")
    writer = csv.writer(csv_f)
    writer.writerow(["t_rel_s", "frame", "class", "confidence",
                     "left", "top", "right", "bottom"])

    counts = {}          # clase -> n detecciones
    frame = 0
    lat_ms = []          # latencia por frame (captura+inferencia), sin warm-up
    t0 = time.monotonic()

    while not stop["flag"]:
        if args.duration > 0 and (time.monotonic() - t0) >= args.duration:
            break

        t_in = time.monotonic()
        img = cam.Capture()
        if img is None:      # timeout de captura
            continue
        dets = net.Detect(img, overlay="box,labels,conf")
        t_out = time.monotonic()

        frame += 1
        if frame > 3:        # descartar warm-up en las stats
            lat_ms.append((t_out - t_in) * 1000.0)

        t_rel = t_out - t0
        for d in dets:
            cls = net.GetClassDesc(d.ClassID)
            counts[cls] = counts.get(cls, 0) + 1
            writer.writerow([round(t_rel, 3), frame, cls,
                             round(d.Confidence, 3), round(d.Left, 1),
                             round(d.Top, 1), round(d.Right, 1),
                             round(d.Bottom, 1)])

        if out is not None:
            out.Render(img)

        if frame % 30 == 0:
            avg = sum(lat_ms[-30:]) / max(len(lat_ms[-30:]), 1)
            print("[test] frame %d | %.1f ms/frame (~%.1f FPS) | red: %.1f FPS"
                  % (frame, avg, 1000.0 / avg if avg else 0.0, net.GetNetworkFPS()))

        if not cam.IsStreaming():
            print("[test] la camara corto el stream")
            break

    dt = time.monotonic() - t0
    csv_f.close()
    if out is not None:
        del out          # cierra/flushea el mp4
    del cam

    print("\n========== RESUMEN ==========")
    print("frames: %d en %.1f s (%.1f FPS efectivos)" % (frame, dt, frame / dt if dt else 0))
    if lat_ms:
        lat_sorted = sorted(lat_ms)
        p50 = lat_sorted[len(lat_sorted) // 2]
        p95 = lat_sorted[int(len(lat_sorted) * 0.95)]
        print("latencia captura+inferencia: p50=%.1f ms  p95=%.1f ms" % (p50, p95))
    print("detecciones por clase: %s" % (counts if counts else "NINGUNA"))
    print("CSV: %s" % csv_path)
    if not args.no_video:
        print("Video: %s" % video_path)
    print("=============================")

    # Codigo de salida util para scripts: 1 si no capturo nada
    sys.exit(0 if frame > 0 else 1)


if __name__ == "__main__":
    main()

# =============================================================================
# COMO CORRERLO (en la Jetson, con el stack ROS APAGADO):
#
#   mkdir -p tools/out
#   docker run --rm -it --runtime nvidia \
#       -v /tmp/argus_socket:/tmp/argus_socket \
#       -v $PWD/tools:/test \
#       capbot-ros-foxy:detector \
#       python3 /test/test_detectnet_camera.py --duration 20
#
#   (guardar este archivo como tools/test_detectnet_camera.py en el repo)
#
# Variantes:
#   --no-video            sin NVENC: aisla camara+GPU, minimo consumo
#   --fps 10 --bitrate 1000000   aun mas suave para la fuente
#   --duration 0          hasta Ctrl+C
#
# Resultados en tools/out/ del host: video .mp4 con overlays + .csv de
# detecciones. Revisar el video para confirmar que TUS obstaculos reales
# aparecen como clases COCO, y el CSV para confianzas/frecuencia.
# =============================================================================