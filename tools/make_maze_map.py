#!/usr/bin/env python3
"""
make_maze_map.py
----------------
Genera el mapa nav2 del laberinto definido en capbot-jetson/controller/sim/game.py
y la base de datos de marcadores ArUco para capbot-ros-foxy.

Produce en src/test_bot/config/ (o --out-dir):
  test_map_maze.pgm        imagen PGM P5 (blanco=libre, negro=ocupado)
  test_map_maze.yaml       descriptor nav2_map_server
  markers_db_maze.yaml     poses de 15 marcadores ArUco (DICT_5X5_250, id 0-14)

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
MARKER_Z     = 0.09    # altura del centro del marcador sobre el suelo

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

def wall_row_y(r):
    """Centro en Y de la pared horizontal entre fila r y r+1."""
    row = WALL_PX + (r + 1) * (CELL_PX + WALL_PX) - WALL_PX + (WALL_PX - 1) / 2.0
    return row_to_y(row)

# ---------------------------------------------------------------------------
# Posiciones de los 15 marcadores ArUco (id 0-14): 10 interiores + 5 exteriores
#
# Convencion RPY identica a generate_aruco_models.py:
#   roll = pi/2, pitch = 0  ->  marcador vertical
#   yaw:  0     = normal apunta hacia el SUR  (-Y)
#         pi    = normal apunta hacia el NORTE (+Y)
#         pi/2  = normal apunta hacia el ESTE  (+X)
#        -pi/2  = normal apunta hacia el OESTE (-X)
#
# Lista hardcodeada: cada marcador va centrado en un tramo de pared que
# existe en MAZE, con la cara hacia una celda abierta (r, c) indicada en
# el comentario. Prioriza paredes interiores para cubrir el centro del
# laberinto; los 5 exteriores rellenan los pasillos perimetrales.
# ---------------------------------------------------------------------------
PI   = math.pi
PI_2 = math.pi / 2.0

def _marker_poses():
    # Centros de las paredes exteriores
    y_north = row_to_y(0.5)                                        #  1.05
    y_south = row_to_y(HEIGHT_PX - WALL_PX + (WALL_PX - 1) / 2.0)  # -1.05
    x_west  = col_to_x(0.5)                                        # -0.875
    x_east  = col_to_x(WIDTH_PX - WALL_PX + (WALL_PX - 1) / 2.0)   #  0.875

    def xc(c):  # x del centro de la columna c
        return cell_center(0, c)[0]

    def yr(r):  # y del centro de la fila r
        return cell_center(r, 0)[1]

    return [
        # (id, x, y, z, roll, pitch, yaw)
        # --- Interiores (10) ---
        (0,  xc(1),         wall_row_y(0), MARKER_Z, PI_2, 0.0,  PI   ),  # pared r0|r1 c1, cara norte -> visto desde (0,1)
        (1,  xc(3),         wall_row_y(0), MARKER_Z, PI_2, 0.0,  0.0  ),  # pared r0|r1 c3, cara sur   -> visto desde (1,3)
        (2,  wall_col_x(1), yr(1),         MARKER_Z, PI_2, 0.0,  PI_2 ),  # pared c1|c2 r1, cara este  -> visto desde (1,2)
        (3,  xc(0),         wall_row_y(1), MARKER_Z, PI_2, 0.0,  0.0  ),  # pared r1|r2 c0, cara sur   -> visto desde (2,0)
        (4,  xc(4),         wall_row_y(1), MARKER_Z, PI_2, 0.0,  0.0  ),  # pared r1|r2 c4, cara sur   -> visto desde (2,4)
        (5,  xc(3),         wall_row_y(2), MARKER_Z, PI_2, 0.0,  0.0  ),  # pared r2|r3 c3, cara sur   -> visto desde (3,3)
        (6,  xc(1),         wall_row_y(3), MARKER_Z, PI_2, 0.0,  PI   ),  # pared r3|r4 c1, cara norte -> visto desde (3,1)
        (7,  wall_col_x(0), yr(4),         MARKER_Z, PI_2, 0.0,  PI_2 ),  # pared c0|c1 r4, cara este  -> visto desde (4,1)
        (8,  wall_col_x(3), yr(4),         MARKER_Z, PI_2, 0.0, -PI_2 ),  # pared c3|c4 r4, cara oeste -> visto desde (4,3)
        (9,  xc(2),         wall_row_y(4), MARKER_Z, PI_2, 0.0,  PI   ),  # pared r4|r5 c2, cara norte -> visto desde (4,2)
        # --- Exteriores (5) ---
        (10, xc(2),         y_north,       MARKER_Z, PI_2, 0.0,  0.0  ),  # Norte, cara sur   -> visto desde (0,2)
        (11, x_east,        yr(1),         MARKER_Z, PI_2, 0.0, -PI_2 ),  # Este, cara oeste  -> visto desde (1,4)
        (12, x_east,        yr(4),         MARKER_Z, PI_2, 0.0, -PI_2 ),  # Este, cara oeste  -> visto desde (4,4)
        (13, xc(1),         y_south,       MARKER_Z, PI_2, 0.0,  PI   ),  # Sur, cara norte   -> visto desde (5,1)
        (14, x_west,        yr(3),         MARKER_Z, PI_2, 0.0,  PI_2 ),  # Oeste, cara este  -> visto desde (3,0)
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
