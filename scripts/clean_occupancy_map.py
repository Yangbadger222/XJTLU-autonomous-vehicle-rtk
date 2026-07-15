#!/usr/bin/env python3
"""Remove tiny disconnected occupied components from a binary PGM occupancy map."""

import argparse
from collections import Counter, deque
from pathlib import Path


NEIGHBORS = (
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
)


def read_pgm(path):
    data = Path(path).read_bytes()
    tokens = []
    index = 0
    while len(tokens) < 4:
        while index < len(data):
            if data[index] == ord("#"):
                index = data.find(b"\n", index)
                if index < 0:
                    raise ValueError("PGM header ends inside a comment")
            elif chr(data[index]).isspace():
                index += 1
            else:
                break
        start = index
        while index < len(data) and not chr(data[index]).isspace():
            index += 1
        if start == index:
            raise ValueError("incomplete PGM header")
        tokens.append(data[start:index])

    magic, width_raw, height_raw, max_value_raw = tokens
    if magic != b"P5":
        raise ValueError("only binary P5 PGM maps are supported")
    width, height, max_value = map(int, (width_raw, height_raw, max_value_raw))
    if not 0 < max_value <= 255:
        raise ValueError("only 8-bit PGM maps are supported")
    if index >= len(data) or not chr(data[index]).isspace():
        raise ValueError("PGM header is not terminated by whitespace")
    index += 1
    if data[index - 1:index + 1] == b"\r\n":
        index += 1
    pixels = bytearray(data[index:])
    if len(pixels) != width * height:
        raise ValueError(
            f"PGM raster has {len(pixels)} bytes, expected {width * height}"
        )
    return width, height, max_value, pixels


def write_pgm(path, width, height, max_value, pixels):
    header = f"P5\n{width} {height}\n{max_value}\n".encode("ascii")
    Path(path).write_bytes(header + bytes(pixels))


def clean_components(width, height, pixels, occupied_max, max_component_cells):
    visited = bytearray(width * height)
    removed_components = 0
    removed_cells = 0

    for start in range(width * height):
        if visited[start] or pixels[start] > occupied_max:
            continue
        visited[start] = 1
        component = []
        queue = deque([start])
        while queue:
            current = queue.popleft()
            component.append(current)
            row, col = divmod(current, width)
            for dr, dc in NEIGHBORS:
                nr, nc = row + dr, col + dc
                if nr < 0 or nr >= height or nc < 0 or nc >= width:
                    continue
                neighbor = nr * width + nc
                if not visited[neighbor] and pixels[neighbor] <= occupied_max:
                    visited[neighbor] = 1
                    queue.append(neighbor)

        if len(component) > max_component_cells:
            continue

        boundary_values = Counter()
        component_set = set(component)
        for current in component:
            row, col = divmod(current, width)
            for dr, dc in NEIGHBORS:
                nr, nc = row + dr, col + dc
                if nr < 0 or nr >= height or nc < 0 or nc >= width:
                    continue
                neighbor = nr * width + nc
                if neighbor not in component_set and pixels[neighbor] > occupied_max:
                    boundary_values[pixels[neighbor]] += 1

        if not boundary_values:
            continue
        # On equal support, prefer the darker non-occupied value (normally unknown=205)
        # over free=254 so cleanup never expands free space speculatively.
        replacement = min(
            boundary_values,
            key=lambda value: (-boundary_values[value], value),
        )
        for current in component:
            pixels[current] = replacement
        removed_components += 1
        removed_cells += len(component)

    return removed_components, removed_cells


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="input binary PGM map")
    parser.add_argument("output", type=Path, help="output binary PGM map")
    parser.add_argument(
        "--max-component-cells",
        type=int,
        default=3,
        help="remove occupied 8-connected components no larger than this (default: 3)",
    )
    parser.add_argument(
        "--occupied-max",
        type=int,
        default=50,
        help="pixel values at or below this are occupied (default: 50)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.input.resolve() == args.output.resolve():
        raise SystemExit("input and output must differ; preserve the original map")
    if args.max_component_cells < 1:
        raise SystemExit("--max-component-cells must be at least 1")
    width, height, max_value, pixels = read_pgm(args.input)
    removed_components, removed_cells = clean_components(
        width,
        height,
        pixels,
        args.occupied_max,
        args.max_component_cells,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_pgm(args.output, width, height, max_value, pixels)
    print(
        f"cleaned {removed_components} components / {removed_cells} occupied cells "
        f"from {width}x{height} map"
    )


if __name__ == "__main__":
    main()
