import re



MARGIN_X = 2.5
MARGIN_Y = 2.5

def parse_axis(line, axis):
    m = re.search(axis + r"([-0-9.]+)", line)
    return float(m.group(1)) if m else None


def find_gcode_bounds(lines):
    """
            Находит границы обработки в G-code
    """

    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')

    patterns = {
        'x': re.compile(r'X([-+]?\d*\.?\d+)'),
        'y': re.compile(r'Y([-+]?\d*\.?\d+)'),
        'z': re.compile(r'Z([-+]?\d*\.?\d+)')
    }

    for line_num, line in enumerate(lines, 1):
        # Пропускаем комментарии
        clean_line = line.split(';')[0].split('(')[0].strip()
        if not clean_line:
            continue

        if line.startswith(("G0", "G1")):

            # Ищем координаты
            x_match = patterns['x'].search(clean_line)
            y_match = patterns['y'].search(clean_line)

            if x_match:
                x_val = float(x_match.group(1))
                min_x = min(min_x, x_val)
                max_x = max(max_x, x_val)

            if y_match:
                y_val = float(y_match.group(1))
                min_y = min(min_y, y_val)
                max_y = max(max_y, y_val)


    # Добавляем поля

    #min_x -= margin_x
    #max_x += margin_x
    #min_y -= margin_y
    #max_y += margin_y

    #print(f"  Скан с полями: X={min_x:.2f}-{max_x:.2f}, Y={min_y:.2f}-{max_y:.2f}")

    return min_x, max_x, min_y, max_y


def find_gcode_bounds_margins(lines, margin_x=MARGIN_X, margin_y=MARGIN_Y): # поиск минимумов с добавлением полей
    min_x, max_x, min_y, max_y = find_gcode_bounds(lines)

    # Добавляем поля

    min_x -= margin_x
    max_x += margin_x
    min_y -= margin_y
    max_y += margin_y

    return min_x, max_x, min_y, max_y

def find_gcode_bounds_margins_reverse(lines, margin_x=MARGIN_X, margin_y=MARGIN_Y): # поиск минимумов с инверсирванным добавлением полей
    min_x, max_x, min_y, max_y = find_gcode_bounds(lines)

    # Убираем поля, восстанавливаем старые координаты

    min_x += margin_x
    max_x -= margin_x
    min_y += margin_y
    max_y -= margin_y

    return min_x, max_x, min_y, max_y


def offset_to_zero_position(lines): # находит минимальную позицию по xy и сдвигает все на минимум, чтобы минимальные поиции были на нуле
    min_x, max_x, min_y, max_y = find_gcode_bounds(lines)
    offset = (-min_x, -min_y)
    return added_xy_shift(lines, offset)


def added_xy_shift(lines, offset=(0.0, 0.0)):
    x_offset = offset[0]
    y_offset = offset[1]
    out_lines = []

    for line in lines:
        out_line = line
        if line.startswith(";") and parse_axis(line, "X") is not None:
            continue
        if line.startswith(("G0", "G1")):
            out_line = "G1 "
            if parse_axis(line, "X") is not None:
                x = float(parse_axis(line, "X")) + x_offset
                out_line += "X" + str(x) + ' '
            if parse_axis(line, "Y") is not None:
                y = float(parse_axis(line, "Y")) + y_offset
                out_line += "Y" + str(y) + ' '
            if parse_axis(line, "Z") is not None:
                z = str(parse_axis(line, "Z"))
                out_line += "Z" + z + ' '
            if parse_axis(line, "E") is not None:
                e = str(parse_axis(line, "E"))
                out_line += "E" + e + ' '
            if parse_axis(line, "F") is not None:
                f = str(parse_axis(line, "F"))
                out_line += "F" + f + ' '
        out_lines.append(out_line)
    return out_lines


def added_z_offset(lines, z_offset=0.0):
    out_lines = []

    for line in lines:
        out_line = line
        if line.startswith(("G0", "G1")):
            out_line = "G1 "
            if parse_axis(line, "X") is not None:
                x = float(parse_axis(line, "X"))
                out_line += "X" + str(x) + ' '
            if parse_axis(line, "Y") is not None:
                y = float(parse_axis(line, "Y"))
                out_line += "Y" + str(y) + ' '
            if parse_axis(line, "Z") is not None:
                z = float(parse_axis(line, "Z")) + z_offset
                out_line += "Z" + str(z) + ' '
            if parse_axis(line, "E") is not None:
                e = str(parse_axis(line, "E"))
                out_line += "E" + e + ' '
            if parse_axis(line, "F") is not None:
                f = str(parse_axis(line, "F"))
                out_line += "F" + f + ' '
            out_line += '\n'
        out_lines.append(out_line)
    return out_lines