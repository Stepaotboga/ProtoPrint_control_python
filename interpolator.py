import re
import math
from configparser import Interpolation
from typing import Any

SEGMENT_LENGTH = 0.1
INTERPOLATION_TYPE = "bicubic"


def parse_axis(line, axis):
    m = re.search(axis + r"([-0-9.]+)", line)
    return float(m.group(1)) if m else None


# ----------------------------
# загрузка heightmap
# ----------------------------
def load_heightmap(filename):

    grid = {}
    current_y = None

    with open(filename, "w", encoding="utf-8") as f:

        for line in f:

            y_match = re.search(r"Y\s*=\s*([-0-9.]+)", line)

            if y_match:
                current_y = float(y_match.group(1))
                grid[current_y] = {}
                continue

            x_match = re.search(r"X\s*=\s*([-0-9.]+).*Z\s*=\s*([-0-9.]+)", line)

            if x_match and current_y is not None:
                x = float(x_match.group(1))
                z = float(x_match.group(2))
                grid[current_y][x] = z

    ys = sorted(grid.keys())
    xs = sorted(next(iter(grid.values())).keys())

    return xs, ys, grid


# ----------------------------
# билинейная интерполяция
# ----------------------------
def interpolate(xs, ys, grid, x, y):

    x1 = max([v for v in xs if v <= x], default=xs[0])
    x2 = min([v for v in xs if v >= x], default=xs[-1])

    y1 = max([v for v in ys if v <= y], default=ys[0])
    y2 = min([v for v in ys if v >= y], default=ys[-1])

    q11 = grid[y1][x1]
    q21 = grid[y1][x2]
    q12 = grid[y2][x1]
    q22 = grid[y2][x2]

    if x1 == x2 and y1 == y2:
        return q11

    if x1 == x2:
        return q11 + (q12 - q11) * (y - y1) / (y2 - y1)

    if y1 == y2:
        return q11 + (q21 - q11) * (x - x1) / (x2 - x1)

    return (
        q11 * (x2 - x) * (y2 - y)
        + q21 * (x - x1) * (y2 - y)
        + q12 * (x2 - x) * (y - y1)
        + q22 * (x - x1) * (y - y1)
    ) / ((x2 - x1) * (y2 - y1))

#бикубическа интерполяция
#--------------

def cubic(p0, p1, p2, p3, t):
    return (
        p1 + 0.5 * t * (p2 - p0 + t * (2*p0 - 5*p1 + 4*p2 - p3 + t * (3*(p1 - p2) + p3 - p0)))
    )


def bicubic(xs, ys, grid, x, y):

    # поиск индексов
    xi = min(range(len(xs)), key=lambda i: abs(xs[i] - x))
    yi = min(range(len(ys)), key=lambda i: abs(ys[i] - y))

    xi = max(1, min(xi, len(xs)-3))
    yi = max(1, min(yi, len(ys)-3))

    # нормированные координаты
    tx = (x - xs[xi]) / (xs[xi+1] - xs[xi])
    ty = (y - ys[yi]) / (ys[yi+1] - ys[yi])

    arr = []

    for j in range(-1,3):

        row = []

        for i in range(-1,3):

            px = xs[xi+i]
            py = ys[yi+j]

            row.append(grid[py][px])

        arr.append(row)

    col = []

    for r in arr:
        col.append(cubic(r[0], r[1], r[2], r[3], tx))

    return cubic(col[0], col[1], col[2], col[3], ty)

def open_and_process_gcode(input_file, output_file):
    lines = []
    with open(input_file) as fin, open(output_file, "w") as fout:
        for line in fin:
            lines.append(line)

        data = process_gcode(lines)
        for i in data:
            fout.write(i)
    print("Done!")





# ----------------------------
# обработка gcode
# ----------------------------
def process_gcode(lines):

    xs, ys, grid = load_heightmap("heightmap.txt")
    x = y = z = e = 0
    absolute_e = True

    output_data = []

    for line in lines:

        if line.startswith("M82"):
            absolute_e = True

        if line.startswith("M83"):
            absolute_e = False

        if line.startswith("G92"):
            ne = parse_axis(line, "E")
            if ne is not None:
                e = ne
            output_data.append(line)
            continue

        if line.startswith(("G0", "G1")):

            is_xyz_move = (parse_axis(line, "X") is not None) or (parse_axis(line, "Y") is not None) or (parse_axis(line, "Z") is not None)
            is_e_move = (parse_axis(line, "E") is not None) and not is_xyz_move

            # Проверка на ретракт
            if is_e_move:
                ne = parse_axis(line, "E")

                if absolute_e:
                    e = ne
                else:
                    e += ne

                output_data.append(line)
                continue

            # Проверка на иные команды если нет движения по осям xyz
            if not is_xyz_move:
                output_data.append(line)
                continue

            nx = parse_axis(line, "X")
            ny = parse_axis(line, "Y")
            nz = parse_axis(line, "Z")
            ne = parse_axis(line, "E")
            nf = parse_axis(line, "F")

            if nx is None:
                nx = x
            if ny is None:
                ny = y
            if nz is None:
                nz = z
            if ne is None:
                ne = e

            dx = nx - x
            dy = ny - y
            dz = nz - z

            dist = math.sqrt(dx * dx + dy * dy)

            steps = max(1, int(dist / SEGMENT_LENGTH))

            for i in range(1, steps + 1):

                t = i / steps

                sx = x + dx * t
                sy = y + dy * t
                sz = z + dz * t

                if INTERPOLATION_TYPE == "bicubic":
                    correction = bicubic(xs, ys, grid, sx, sy)
                else:
                    correction = interpolate(xs, ys, grid, sx, sy)

                if absolute_e:
                    se = e + (ne - e) * t
                else:
                    se = (ne / steps)

                gline = f"G1 X{sx:.3f} Y{sy:.3f} Z{sz + correction:.4f}"

                if ne is not None:
                    gline += f' E{se:.5f}'

                if nf and i == steps:
                    gline += f" F{nf}"

                output_data.append(gline + "\n")

            x, y, z, e = nx, ny, nz, ne

        else:
            output_data.append(line)

    return output_data


# ----------------------------
# main
# ----------------------------
def main():

    open_and_process_gcode(
        "reed_plate_2.gcode",
        "output_corrected.gcode",
    )


if __name__ == "__main__":
    main()