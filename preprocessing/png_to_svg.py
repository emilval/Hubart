"""Convert black line drawings on a white background from PNG to SVG."""

import argparse
from pathlib import Path

import cv2
import numpy as np
from skimage.morphology import skeletonize


def _skeleton_to_paths(skeleton: np.ndarray) -> list[list[tuple[int, int]]]:
	pixels = {(int(x), int(y)) for y, x in np.argwhere(skeleton)}
	neighbors: dict[tuple[int, int], list[tuple[int, int]]] = {}
	for x, y in pixels:
		adjacent = []
		for delta_y in (-1, 0, 1):
			for delta_x in (-1, 0, 1):
				if delta_x == 0 and delta_y == 0:
					continue
				neighbor = (x + delta_x, y + delta_y)
				if neighbor not in pixels:
					continue
				if delta_x and delta_y and (
					(x + delta_x, y) in pixels or (x, y + delta_y) in pixels
				):
					continue
				adjacent.append(neighbor)
		neighbors[(x, y)] = sorted(adjacent)

	visited_edges: set[frozenset[tuple[int, int]]] = set()
	paths: list[list[tuple[int, int]]] = []

	def walk(start: tuple[int, int], first: tuple[int, int]) -> list[tuple[int, int]]:
		path = [start]
		previous, current = start, first
		while True:
			edge = frozenset((previous, current))
			if edge in visited_edges:
				break
			visited_edges.add(edge)
			path.append(current)
			if current == start or len(neighbors[current]) != 2:
				break
			next_pixels = [pixel for pixel in neighbors[current] if pixel != previous]
			if not next_pixels:
				break
			previous, current = current, next_pixels[0]
		return path

	for pixel, adjacent in neighbors.items():
		if not adjacent:
			paths.append([pixel])
		elif len(adjacent) != 2:
			for neighbor in adjacent:
				if frozenset((pixel, neighbor)) not in visited_edges:
					paths.append(walk(pixel, neighbor))

	for pixel, adjacent in neighbors.items():
		for neighbor in adjacent:
			if frozenset((pixel, neighbor)) not in visited_edges:
				paths.append(walk(pixel, neighbor))
	return paths


def png_to_svg(
	input_path: str | Path,
	output_path: str | Path | None = None,
	threshold: int = 127,
	simplify: float = 1.0,
) -> Path:
	"""Trace dark regions in a PNG and save their centerlines as SVG strokes.

	``threshold`` controls which grayscale pixels count as black. ``simplify``
	is the contour approximation tolerance in pixels; use 0 to disable it.
	"""
	source = Path(input_path)
	images_dir = Path(__file__).resolve().parent.parent / "images"
	destination = Path(output_path) if output_path else images_dir / f"{source.stem}.svg"

	if not 0 <= threshold <= 255:
		raise ValueError("threshold must be between 0 and 255")
	if simplify < 0:
		raise ValueError("simplify must be non-negative")
	if source.resolve() == destination.resolve():
		raise ValueError("input and output paths must be different")

	image = cv2.imread(str(source), cv2.IMREAD_GRAYSCALE)
	if image is None:
		raise FileNotFoundError(f"Could not read PNG image: {source}")

	height, width = image.shape
	_, binary = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY_INV)
	skeleton = skeletonize(binary > 0)

	path_data = []
	for points in _skeleton_to_paths(skeleton):
		closed = len(points) > 2 and points[0] == points[-1]
		if closed:
			points = points[:-1]
		contour = np.asarray(points, dtype=np.int32).reshape(-1, 1, 2)
		if simplify and len(points) > 2:
			contour = cv2.approxPolyDP(contour, simplify, closed)
		points = contour.reshape(-1, 2)
		commands = [f"M {points[0][0]} {points[0][1]}"]
		commands.extend(f"L {x} {y}" for x, y in points[1:])
		if closed:
			commands.append("Z")
		path_data.append(" ".join(commands))

	destination.parent.mkdir(parents=True, exist_ok=True)
	svg_paths = "\n".join(f'  <path d="{data}" />' for data in path_data)
	svg = (
		'<?xml version="1.0" encoding="UTF-8"?>\n'
		f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}px" '
		f'height="{height}px" viewBox="0 0 {width} {height}">\n'
		'  <g fill="none" stroke="black" stroke-width="1" '
		f'stroke-linecap="round" stroke-linejoin="round">\n{svg_paths}\n  </g>\n'
		'</svg>\n'
	)
	destination.write_text(svg, encoding="utf-8")
	return destination


def main() -> None:
	parser = argparse.ArgumentParser(
		description="Convert a black-on-white PNG drawing to SVG."
	)
	parser.add_argument("input_png", type=Path, help="PNG image to convert")
	parser.add_argument(
		"output_svg", type=Path, nargs="?",
		help="SVG output path (defaults to the repo's images directory)",
	)
	parser.add_argument(
		"--threshold", type=int, default=127,
		help="grayscale cutoff from 0 (black) to 255 (white), default: 127",
	)
	parser.add_argument(
		"--simplify", type=float, default=1.0,
		help="contour simplification tolerance in pixels, default: 1.0",
	)
	args = parser.parse_args()

	result = png_to_svg(args.input_png, args.output_svg, args.threshold, args.simplify)
	print(f"Saved SVG: {result}")


if __name__ == "__main__":
	main()
