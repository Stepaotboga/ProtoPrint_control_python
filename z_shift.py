import re



MARGIN_X = 2.5
MARGIN_Y = 2.5

def parse_axis(line, axis):
    m = re.search(axis + r"([-0-9.]+)", line)
    return float(m.group(1)) if m else None


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
        out_lines.append(out_line)
    return out_lines


input_file = "output_main.gcode"
output_file = "run.gcode"
offset_z = -7.1
with open(input_file) as fin, open(output_file, "w") as fout:
    lines = []
    for line in fin:
        lines.append(line)

    data = added_z_offset(lines, offset_z)
    for i in data:
        fout.write(i + '\n')
print("Done!")