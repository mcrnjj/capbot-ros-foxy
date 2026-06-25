#!/usr/bin/env python3
"""
make_maze_map.py
----------------
Genera el mapa nav2 del laberinto definido en capbot-jetson/controller/sim/game.py
y la base de datos de marcadores ArUco para capbot-ros-foxy.

Produce en src/test_bot/config/ (o --out-dir):
  test_map_maze.pgm        imagen PGM P5 (blanco=libre, negro=ocupado)
  test_map_maze.yaml       descriptor nav2_map_server
  markers_db_maze.yaml     poses de 5 marcadores ArUco (DICT_5X5_250, id 0-4)

Uso:
  python3 make_maze_map.py
  python3 make_maze_map.py --out-dir /ruta/a/capbot-ros-foxy/src/test_bot/config --force

Lanzar luego con:
  ros2 launch test_bot real_robot.launch.py map_name:=maze
"""
import argparse
import math
import os
import sys

# ---------------------------------------------------------------------------
# Definicion del laberinto (fuente: capbot-jetson/controller/sim/game.py)
# Cada celda: [UP, DOWN, LEFT, RIGHT] -- 1 = hay pared, 0 = pasaje abierto
# ---------------------------------------------------------------------------
MAZE = [
    [[1, 0, 1, 0], [1, 1, 0, 0], [1, 1, 0, 0], [1, 1, 0, 0], [1, 0, 0, 1]],
    [[0, 1, 1, 0], [1, 0, 0, 1], [1, 1, 1, 0], [1, 1, 0, 0], [0, 1, 0, 1]],
    [[1, 0, 1, 1], [0, 0, 1, 1], [1, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1]],
    [[0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 0], [1, 0, 0, 1], [0, 0, 1, 1]],
    [[0, 0, 1, 1], [1, 0, 1, 0], [0, 1, 0, 0], [0, 1, 0, 1], [0, 0, 1, 1]],
    [[0, 1, 1, 0], [0, 1, 0, 1], [1, 1, 1, 0], [1, 1, 0, 0], [0, 1, 0, 1]],
]

ROWS = 6
COLS = 5

# ---------------------------------------------------------------------------
# Parametros del mapa
# ---------------------------------------------------------------------------
RESOLUTION   = 0.025   # m/pixel
CELL_SIZE_M  = 0.30    # 30 cm por celda (igual que CELL_SIZE_CM en game.py)
WALL_SIZE_M  = 0.05    # 5 cm de espesor de pared

CELL_PX = round(CELL_SIZE_M / RESOLUTION)   # 12 px = 30 cm
WALL_PX = round(WALL_SIZE_M / RESOLUTION)   # 2 px  = 5 cm

# Dimensiones totales de la imagen:
#   WALL_PX borde externo + COLS * (CELL_PX + WALL_PX) [cada celda + su pared derecha/inferior]
WIDTH_PX  = WALL_PX + COLS * (CELL_PX + WALL_PX)   # 72 px = 1.80 m
HEIGHT_PX = WALL_PX + ROWS * (CELL_PX + WALL_PX)   # 86 px = 2.15 m

ORIGIN_X = -(WIDTH_PX  * RESOLUTION) / 2.0    # -0.9000 m
ORIGIN_Y = -(HEIGHT_PX * RESOLUTION) / 2.0    # -1.0750 m

MAP_NAME     = "maze"
ARUCO_DICT   = "DICT_5X5_250"
MARKER_SIZE  = 0.10    # metros (lado del cuadrado negro del marcador)
MARKER_Z     = 0.15    # altura del centro del marcador sobre el suelo

# ---------------------------------------------------------------------------
# Helpers de conversion pixel <-> coordenada de mapa
# ---------------------------------------------------------------------------
# Convencion nav2: fila 0 del PGM = borde superior = y_max del mapa.
# map_x(col) = ORIGIN_X + (col + 0.5) * RESOLUTION
# map_y(row) = ORIGIN_Y + (HEIGHT_PX - row - 0.5) * RESOLUTION

def col_to_x(col):
    return ORIGIN_X + (col + 0.5) * RESOLUTION

def row_to_y(row):
    return ORIGIN_Y + (HEIGHT_PX - row - 0.5) * RESOLUTION

def cell_center(r, c):
    """Coordenadas en metros del centro de la celda (r, c)."""
    col = WALL_PX + c * (CELL_PX + WALL_PX) + CELL_PX // 2
    row = WALL_PX + r * (CELL_PX + WALL_PX) + CELL_PX // 2
    return col_to_x(col), row_to_y(row)

def wall_col_x(c):
    """Centro en X de la pared vertical entre columna c y c+1."""
    col = WALL_PX + (c + 1) * (CELL_PX + WALL_PX) - WALL_PX + (WALL_PX - 1) / 2.0
    return ORIGIN_X + (col + 0.5) * RESOLUTION

# ---------------------------------------------------------------------------
# Posiciones de los 5 marcadores ArUco (id 0-4)
#
# Convencion RPY identica a generate_aruco_models.py:
#   roll = pi/2, pitch = 0  ->  marcador vertical
#   yaw:  0     = normal apunta hacia el SUR  (-Y)
#         pi    = normal apunta hacia el NORTE (+Y)
#         pi/2  = normal apunta hacia el ESTE  (+X)
#        -pi/2  = normal apunta hacia el OESTE (-X)
#
# Notas de ubicacion fisica:
#   id 0  Pared norte exterior, centrado en columna 2 -> robot lo ve desde filas 0-1
#   id 1  Pared sur exterior, centrado en columna 1   -> robot lo ve al alcanzar zona objetivo
#   id 2  Pared oeste exterior, centrado en fila 1    -> cubre el pasillo izquierdo
#   id 3  Pared este exterior, centrado en fila 4     -> cubre el pasillo derecho
#   id 4  Pared interior entre c=1 y c=2 en fila 2   -> cobertura en el centro del laberinto
# ---------------------------------------------------------------------------
PI   = math.pi
PI_2 = math.pi / 2.0

def _marker_poses():
    # Pared norte exterior: row 0..1, centro row=0.5
    y_north = row_to_y(0.5)  # 1.05
    # Pared sur exterior: row 84..85, centro row=84.5
    y_south = row_to_y(HEIGHT_PX - WALL_PX + (WALL_PX - 1) / 2.0)  # -1.05
    # Pared oeste exterior: col 0..1, centro col=0.5
    x_west  = col_to_x(0.5)   # -0.875
    # Pared este exterior: col 70..71, centro col=70.5
    x_east  = col_to_x(WIDTH_PX - WALL_PX + (WALL_PX - 1) / 2.0)  # 0.875

    # Pared interior entre c=1 y c=2: cols 28..29, centro col=28.5
    wall_col_c1c2 = WALL_PX + 2 * (CELL_PX + WALL_PX) - WALL_PX + (WALL_PX - 1) / 2.0  # 28.5
    x_inner  = col_to_x(wall_col_c1c2)  # -0.175

    _, y_row1 = cell_center(1, 0)   # y del centro de la fila 1
    _, y_row4 = cell_center(4, 0)   # y del centro de la fila 4
    x_col1, _ = cell_center(0, 1)   # x del centro de la columna 1
    x_col2, _ = cell_center(0, 2)   # x del centro de la columna 2
    _, y_row2  = cell_center(2, 0)  # y del centro de la fila 2

    return [
        # (id, x, y, z, roll, pitch, yaw)
        (0,  x_col2,  y_north, MARKER_Z,  PI_2, 0.0,  0.0  ),  # Norte, cara sur
        (1,  x_col1,  y_south, MARKER_Z,  PI_2, 0.0,  PI   ),  # Sur, cara norte
        (2,  x_west,  y_row1,  MARKER_Z,  PI_2, 0.0,  PI_2 ),  # Oeste, cara este
        (3,  x_east,  y_row4,  MARKER_Z,  PI_2, 0.0, -PI_2 ),  # Este, cara oeste
        (4,  x_inner, y_row2,  MARKER_Z,  PI_2, 0.0,  PI_2 ),  # Interior, cara este
    ]

MARKER_POSES = _marker_poses()

# ---------------------------------------------------------------------------
# Generacion de imagen PGM
# ---------------------------------------------------------------------------
FREE     = 255
OCCUPIED = 0

def build_maze_grid():
    """Genera el grid PGM: arranca todo ocupado y talla los espacios libres."""
    W, H = WIDTH_PX, HEIGHT_PX
    data = bytearray(W * H)   # todo negro = ocupado

    def fill(r0, c0, r1, c1, val):
        for row in range(r0, r1):
            base = row * W
            for col in range(c0, c1):
                data[base + col] = val

    for r in range(ROWS):
        for c in range(COLS):
            cr = WALL_PX + r * (CELL_PX + WALL_PX)   # fila inicial del interior
            cc = WALL_PX + c * (CELL_PX + WALL_PX)   # columna inicial del interior

            # Interior de la celda
            fill(cr, cc, cr + CELL_PX, cc + CELL_PX, FREE)

            # Pasaje a la derecha (si no hay pared RIGHT y hay celda adyacente)
            if MAZE[r][c][3] == 0 and c + 1 < COLS:
                fill(cr, cc + CELL_PX, cr + CELL_PX, cc + CELL_PX + WALL_PX, FREE)

            # Pasaje hacia abajo (si no hay pared DOWN y hay celda adyacente)
            if MAZE[r][c][1] == 0 and r + 1 < ROWS:
                fill(cr + CELL_PX, cc, cr + CELL_PX + WALL_PX, cc + CELL_PX, FREE)

    return W, H, bytes(data)


def write_pgm(path, W, H, data):
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (W, H))
        f.write(data)


# ---------------------------------------------------------------------------
# YAML del mapa
# ---------------------------------------------------------------------------
def write_map_yaml(path, pgm_name):
    with open(path, "w") as f:
        f.write(
            f"image: {pgm_name}\n"
            f"resolution: {RESOLUTION}\n"
            f"origin: [{ORIGIN_X:.4f}, {ORIGIN_Y:.4f}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )


# ---------------------------------------------------------------------------
# YAML de marcadores ArUco
# ---------------------------------------------------------------------------
def write_markers_db(path):
    with open(path, "w") as f:
        f.write(f"aruco_dict: {ARUCO_DICT}\n")
        f.write(f"marker_size: {MARKER_SIZE}\n")
        f.write("markers:\n")
        for mid, x, y, z, roll, pitch, yaw in MARKER_POSES:
            f.write(
                f"- id: {mid}\n"
                f"  frame_id: map\n"
                f"  x: {x:.4f}\n"
                f"  y: {y:.4f}\n"
                f"  z: {z}\n"
                f"  roll: {roll:.16f}\n"
                f"  pitch: {pitch:.1f}\n"
                f"  yaw: {yaw:.16f}\n"
            )


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default=None,
                     help="Directorio de salida (def ../src/test_bot/config relativo a este script)")
    ap.add_argument("--force", action="store_true", help="Sobrescribir si ya existen")
    args = ap.parse_args()

    out_dir = args.out_dir or os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "src", "test_bot", "config"))
    os.makedirs(out_dir, exist_ok=True)

    pgm_name  = f"test_map_{MAP_NAME}.pgm"
    pgm_path  = os.path.join(out_dir, pgm_name)
    yaml_path = os.path.join(out_dir, f"test_map_{MAP_NAME}.yaml")
    db_path   = os.path.join(out_dir, f"markers_db_{MAP_NAME}.yaml")

    for p in (pgm_path, yaml_path, db_path):
        if os.path.exists(p) and not args.force:
            print(f"[X] {p} ya existe (usa --force para sobrescribir)")
            sys.exit(1)

    W, H, grid = build_maze_grid()
    print(f"[i] Laberinto {COLS}x{ROWS} celdas -> imagen {W}x{H} px "
          f"({W*RESOLUTION:.2f} x {H*RESOLUTION:.2f} m @ {RESOLUTION} m/px)")

    write_pgm(pgm_path, W, H, grid)
    print(f"[OK] PGM:        {pgm_path}")

    write_map_yaml(yaml_path, pgm_name)
    print(f"[OK] YAML mapa:  {yaml_path}  "
          f"(origin=[{ORIGIN_X:.4f}, {ORIGIN_Y:.4f}, 0.0])")

    write_markers_db(db_path)
    print(f"[OK] markers_db: {db_path}")
    print(f"     {len(MARKER_POSES)} marcadores:")
    for mid, x, y, z, roll, pitch, yaw in MARKER_POSES:
        print(f"       id={mid}  ({x:+.4f}, {y:+.4f}, {z:.2f})  yaw={math.degrees(yaw):+.1f} deg")

    print(f"\n==== LISTO ====")
    print(f"  ros2 launch test_bot real_robot.launch.py map_name:={MAP_NAME}")


if __name__ == "__main__":
    main()
