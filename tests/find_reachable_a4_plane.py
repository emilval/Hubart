from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control.inverse_kinematics import get_servo_angles


A4_WIDTH_M = 0.210
A4_HEIGHT_M = 0.297
GRID_STEP_M = 0.01

# Keep the same fixed-plane assumption as the current mapping code: points live in a
# YZ plane with constant X. Search for a plane translation that keeps every A4 point
# inside the robot's reachable workspace.


@dataclass
class PlaneCandidate:
    x0: float
    y0: float
    z0: float
    success_count: int
    total_points: int
    min_margin_deg: float

    @property
    def success_ratio(self) -> float:
        return self.success_count / self.total_points if self.total_points else 0.0

    def as_tuple(self) -> tuple[float, float, float, int, int, float]:
        return (
            self.x0,
            self.y0,
            self.z0,
            self.success_count,
            self.total_points,
            self.min_margin_deg,
        )


def make_a4_grid(step_m: float = GRID_STEP_M) -> List[tuple[float, float]]:
    """Return a dense grid of (u, v) paper coordinates for an A4 sheet."""
    u_points = np.arange(0.0, A4_WIDTH_M + step_m * 0.5, step_m)
    v_points = np.arange(0.0, A4_HEIGHT_M + step_m * 0.5, step_m)
    grid: List[tuple[float, float]] = []
    for u in u_points:
        for v in v_points:
            grid.append((float(u), float(v)))
    return grid


def plane_point(u: float, v: float, x0: float, y0: float, z0: float) -> tuple[float, float, float]:
    """Map a 2D A4 point to a candidate flat plane in the current YZ-wall convention."""
    x = x0
    y = y0 - u
    z = z0 - v
    return (x, y, z)


def evaluate_plane(x0: float, y0: float, z0: float, points: Sequence[tuple[float, float]]) -> tuple[int, float]:
    """Return (success_count, min_margin_deg) over all A4 points for a candidate plane."""
    success_count = 0
    min_margin_deg = float("inf")

    # Use the exact same IK solver as the robot motion pipeline.
    prev = (0.0, 0.0, 0.0)

    for u, v in points:
        target = plane_point(u, v, x0, y0, z0)
        try:
            angles = get_servo_angles(target, prev)
        except Exception:
            # If the IK fails for this target, it is outside the solver's reachable workspace.
            continue

        # Check against the joint limits used by the IK model.
        limits = [
            (-math.pi / 2, math.pi / 2),
            (-math.pi / 2, math.pi / 3),
            (-math.pi / 2, math.pi / 2),
        ]

        margin = float("inf")
        for angle_deg, (lower, upper) in zip(angles, limits):
            angle_rad = math.radians(angle_deg)
            if angle_rad < lower:
                margin = min(margin, abs(angle_rad - lower))
            elif angle_rad > upper:
                margin = min(margin, abs(angle_rad - upper))

        if math.isinf(margin):
            success_count += 1
            min_margin_deg = min(min_margin_deg, 9999.0)
        else:
            # We allow some margin, but keep the closest bound distance in degrees.
            min_margin_deg = min(min_margin_deg, math.degrees(margin))

        prev = angles

    return success_count, min_margin_deg


def find_best_a4_plane(
    grid_step_m: float = GRID_STEP_M,
    quick: bool = False,
) -> PlaneCandidate:
    grid = make_a4_grid(step_m=grid_step_m)
    total_points = len(grid)

    best: PlaneCandidate | None = None

    if quick:
        x_candidates = np.linspace(0.20, 0.42, 6)
        y_candidates = np.linspace(-0.10, 0.10, 5)
        z_candidates = np.linspace(0.20, 0.40, 6)
    else:
        x_candidates = np.linspace(0.20, 0.42, 24)
        y_candidates = np.linspace(-0.15, 0.15, 25)
        z_candidates = np.linspace(0.18, 0.45, 28)

    total_candidates = len(x_candidates) * len(y_candidates) * len(z_candidates)
    checked = 0

    for x0 in x_candidates:
        for y0 in y_candidates:
            for z0 in z_candidates:
                checked += 1
                success_count, min_margin_deg = evaluate_plane(float(x0), float(y0), float(z0), grid)
                candidate = PlaneCandidate(
                    x0=float(x0),
                    y0=float(y0),
                    z0=float(z0),
                    success_count=success_count,
                    total_points=total_points,
                    min_margin_deg=min_margin_deg,
                )

                if best is None or candidate.success_ratio > best.success_ratio or (
                    math.isclose(candidate.success_ratio, best.success_ratio, abs_tol=1e-9)
                    and candidate.min_margin_deg > best.min_margin_deg
                ):
                    best = candidate

                if quick and checked % 20 == 0:
                    print(f"progress: {checked}/{total_candidates} candidates checked")

    if best is None:
        raise RuntimeError("No candidate plane was evaluated.")

    return best


def main() -> None:
    parser = argparse.ArgumentParser(description="Find a reachable fixed-plane mapping for an A4 sheet.")
    parser.add_argument("--quick", action="store_true", help="Use a coarse fast search to print a likely candidate quickly.")
    parser.add_argument("--grid-step", type=float, default=GRID_STEP_M, help="Grid spacing in meters for the A4 search space.")
    args = parser.parse_args()

    best = find_best_a4_plane(grid_step_m=args.grid_step, quick=args.quick)
    print("Best fixed-plane candidate for an A4 sheet:")
    print(f"  x0 = {best.x0:.4f} m")
    print(f"  y0 = {best.y0:.4f} m")
    print(f"  z0 = {best.z0:.4f} m")
    print(f"  reachable points = {best.success_count}/{best.total_points}")
    print(f"  success ratio = {best.success_ratio:.2%}")
    print(f"  min margin to IK limits = {best.min_margin_deg:.3f} deg")

    sample_points = [
        (0.0, 0.0),
        (A4_WIDTH_M / 2, A4_HEIGHT_M / 2),
        (A4_WIDTH_M, A4_HEIGHT_M),
    ]
    print("\nSample target points on candidate plane:")
    for u, v in sample_points:
        p = plane_point(u, v, best.x0, best.y0, best.z0)
        print(f"  paper ({u:.3f}, {v:.3f}) -> world {p}")


if __name__ == "__main__":
    main()
