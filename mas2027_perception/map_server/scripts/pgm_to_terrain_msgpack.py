#!/usr/bin/env python3
"""Convert the repository's Nav2 PGM/YAML map to the HW terrain-map subset.

The result deliberately contains only FLAT and OBSTACLE labels.  Directional
terrain cannot be inferred from an occupancy image, so direction is zero.
"""
import argparse
import re
import struct
from pathlib import Path

from PIL import Image


def read_yaml(path: Path):
    text = path.read_text(encoding="utf-8")
    resolution = float(re.search(r"^resolution:\s*([0-9.eE+-]+)", text, re.M).group(1))
    origin = re.search(r"^origin:\s*\[([^]]+)\]", text, re.M)
    origin_values = [float(x.strip()) for x in origin.group(1).split(",")] if origin else [0.0, 0.0, 0.0]
    image = re.search(r"^image:\s*(\S+)", text, re.M).group(1)
    occupied = float(re.search(r"^occupied_thresh:\s*([0-9.eE+-]+)", text, re.M).group(1))
    negate_match = re.search(r"^negate:\s*(\d+)", text, re.M)
    return image, resolution, origin_values, occupied, bool(int(negate_match.group(1))) if negate_match else False


def pack_map(width, height, resolution, terrain, direction):
    # msgpack map with string keys and uint/int scalar values.
    def blob(value):
        return (bytes([0x90 | len(value)]) if len(value) < 16 else b"\xdd" + struct.pack(">I", len(value))) + b"".join(bytes([item]) for item in value)

    def string(value):
        encoded = value.encode()
        return bytes([0xa0 | len(encoded)]) + encoded

    result = bytearray(b"\x85")
    for key, value in (("width", width), ("height", height), ("resolution", resolution),
                       ("terrain", blob(bytes(terrain))), ("direction", blob(bytes(direction)))):
        result += string(key)
        if isinstance(value, float):
            result += b"\xcb" + struct.pack(">d", value)
        elif isinstance(value, int):
            result += b"\xd2" + struct.pack(">i", value)
        else:
            result += value
    return bytes(result)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("yaml", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    image_name, resolution, _, occupied_thresh, negate = read_yaml(args.yaml)
    image_path = args.yaml.parent / image_name
    image = Image.open(image_path).convert("L")
    # ROS OccupancyGrid row zero is the bottom row; PGM row zero is the top.
    pixels = list(image.transpose(Image.Transpose.FLIP_TOP_BOTTOM).getdata())
    terrain = []
    for pixel in pixels:
        probability = pixel / 255.0 if negate else (255 - pixel) / 255.0
        occupied = probability >= occupied_thresh
        if negate:
            occupied = not occupied
        terrain.append(1 if occupied else 0)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(pack_map(image.width, image.height, resolution, terrain, [0] * len(terrain)))
    print(f"wrote {args.output} ({image.width}x{image.height}, {resolution} m/px)")


if __name__ == "__main__":
    main()
