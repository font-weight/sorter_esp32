"""Fit a planar camera calibration from measured CSV correspondences.

No camera, serial port or motor is opened. NumPy is the only dependency.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


class CalibrationError(ValueError):
    pass


def _points(value, name):
    array = np.asarray(value, dtype=float)
    if array.ndim != 2 or array.shape[1] != 2 or not np.all(np.isfinite(array)):
        raise CalibrationError(f"{name}: expected finite Nx2 points")
    return array


def convex_hull(points):
    pts = sorted(set(map(tuple, _points(points, "hull"))))
    if len(pts) < 3:
        raise CalibrationError("At least three distinct hull points required")

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    result = np.asarray(lower[:-1] + upper[:-1], dtype=float)
    if len(result) < 3:
        raise CalibrationError("Points are collinear")
    return result


def inside_convex(points, polygon, tolerance=1e-8):
    pts, poly = _points(points, "points"), _points(polygon, "polygon")
    if len(poly) < 3:
        raise CalibrationError("Polygon requires at least three vertices")
    edges = np.roll(poly, -1, axis=0) - poly
    rel = pts[:, None, :] - poly[None, :, :]
    cross = edges[None, :, 0] * rel[:, :, 1] - edges[None, :, 1] * rel[:, :, 0]
    return np.all(cross >= -tolerance, axis=1) | np.all(cross <= tolerance, axis=1)


def normalize_points(points):
    points = _points(points, "normalization")
    center = points.mean(axis=0)
    mean_distance = np.linalg.norm(points - center, axis=1).mean()
    if mean_distance < 1e-12:
        raise CalibrationError("Points have zero spatial extent")
    scale = math.sqrt(2) / mean_distance
    transform = np.array([[scale, 0, -scale * center[0]],
                          [0, scale, -scale * center[1]], [0, 0, 1]], dtype=float)
    return (points - center) * scale, transform


def fit_homography(image_points, world_points):
    src, dst = _points(image_points, "image"), _points(world_points, "world")
    if len(src) != len(dst) or len(src) < 4:
        raise CalibrationError("At least four corresponding points required")
    if len(set(map(tuple, src))) != len(src) or len(set(map(tuple, dst))) != len(dst):
        raise CalibrationError("Duplicate calibration points")
    convex_hull(src)
    convex_hull(dst)
    src_n, t_src = normalize_points(src)
    dst_n, t_dst = normalize_points(dst)
    rows = []
    for (u, v), (x, y) in zip(src_n, dst_n):
        rows.append([-u, -v, -1, 0, 0, 0, x * u, x * v, x])
        rows.append([0, 0, 0, -u, -v, -1, y * u, y * v, y])
    a = np.asarray(rows)
    _, singular, vt = np.linalg.svd(a, full_matrices=True)
    # Only eight independent constraints identify H up to scale. With noisy
    # overdetermined data the ninth singular value is normally nonzero.
    independent_min = singular[7]
    if independent_min <= singular[0] * 1e-10:
        raise CalibrationError("Degenerate or nearly degenerate point geometry")
    condition = float(singular[0] / independent_min)
    h = np.linalg.inv(t_dst) @ vt[-1].reshape(3, 3) @ t_src
    scale = h[2, 2] if abs(h[2, 2]) > 1e-12 else np.linalg.norm(h)
    h /= scale
    if abs(np.linalg.det(h)) < 1e-14 or not np.all(np.isfinite(h)):
        raise CalibrationError("Homography is singular")
    project_points(h, src)
    return h, condition


def project_points(homography, points):
    pts = _points(points, "projection")
    h = np.asarray(homography, dtype=float)
    if h.shape != (3, 3) or not np.all(np.isfinite(h)):
        raise CalibrationError("Invalid homography")
    projected = np.c_[pts, np.ones(len(pts))] @ h.T
    if np.any(np.abs(projected[:, 2]) < 1e-10):
        raise CalibrationError("Projection approaches the horizon")
    result = projected[:, :2] / projected[:, 2:]
    if not np.all(np.isfinite(result)):
        raise CalibrationError("Non-finite projection")
    return result


def error_metrics(errors):
    values = np.asarray(errors, dtype=float)
    return {"count": int(len(values)), "mean_mm": float(values.mean()),
            "rms_mm": float(np.sqrt(np.mean(values ** 2))),
            "p95_mm": float(np.percentile(values, 95)), "max_mm": float(values.max())}


def read_points(path):
    with Path(path).open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"split", "u", "v", "x_mm", "y_mm"}
        if not required.issubset(reader.fieldnames or []):
            raise CalibrationError("CSV requires split,u,v,x_mm,y_mm columns")
        rows = []
        for line, row in enumerate(reader, 2):
            split = (row.get("split") or "").strip()
            if split not in ("train", "test"):
                raise CalibrationError(f"CSV line {line}: split must be train or test")
            try:
                values = [float(row[k]) for k in ("u", "v", "x_mm", "y_mm")]
            except (ValueError, TypeError) as exc:
                raise CalibrationError(f"CSV line {line}: invalid coordinates") from exc
            if not all(math.isfinite(v) for v in values):
                raise CalibrationError(f"CSV line {line}: non-finite coordinates")
            rows.append({"split": split, "image": values[:2], "world": values[2:]})
    return rows


def build_calibration(rows, *, width=320, height=240, plane_height_mm=10.0,
                      provenance="unverified", valid_rect=None,
                      max_test_error_mm=2.0, calibration_id=None):
    if provenance not in ("synthetic", "measured", "unverified"):
        raise CalibrationError("Invalid provenance")
    if (isinstance(width, bool) or isinstance(height, bool) or
            not isinstance(width, int) or not isinstance(height, int) or
            not 2 <= width <= 65535 or not 2 <= height <= 65535):
        raise CalibrationError("Invalid image dimensions")
    if not math.isfinite(plane_height_mm) or plane_height_mm < 0:
        raise CalibrationError("plane_height_mm must be finite and nonnegative")
    if not math.isfinite(max_test_error_mm) or max_test_error_mm <= 0:
        raise CalibrationError("max_test_error_mm must be positive")
    train = [r for r in rows if r["split"] == "train"]
    test = [r for r in rows if r["split"] == "test"]
    if len(train) < 6 or len(test) < 3:
        raise CalibrationError("Use at least 6 training and 3 independent test points")
    image = _points([r["image"] for r in rows], "all image")
    world = _points([r["world"] for r in rows], "all world")
    if len(set(map(tuple, image))) != len(rows) or len(set(map(tuple, world))) != len(rows):
        raise CalibrationError("Train/test points must be distinct; duplicate positions leak validation data")
    if np.any(image < 0) or np.any(image[:, 0] >= width) or np.any(image[:, 1] >= height):
        raise CalibrationError("Image point is outside the configured frame")
    src, dst = np.asarray([r["image"] for r in train]), np.asarray([r["world"] for r in train])
    test_src, test_dst = np.asarray([r["image"] for r in test]), np.asarray([r["world"] for r in test])
    h, condition = fit_homography(src, dst)
    hull = convex_hull(src)
    if valid_rect is not None:
        u0, v0, u1, v1 = map(float, valid_rect)
        if not all(math.isfinite(v) for v in (u0, v0, u1, v1)) or not u0 < u1 or not v0 < v1:
            raise CalibrationError("Invalid rectangle")
        support = np.array([[u0, v0], [u1, v0], [u1, v1], [u0, v1]])
        if not np.all(inside_convex(support, hull)):
            raise CalibrationError("Valid rectangle must lie inside the training convex hull")
    else:
        support = hull
    if len(support) > 16:
        raise CalibrationError("Support has more than 16 vertices; use an inscribed --valid-rect")
    if not np.all(inside_convex(test_src, support)):
        raise CalibrationError("All holdout points must lie within the chosen valid area")
    # The projective denominator is affine. Same nonzero sign at all convex
    # vertices guarantees no horizon crosses the entire support polygon.
    den = np.c_[support, np.ones(len(support))] @ h[2]
    if np.min(np.abs(den)) < 1e-8 or (np.min(den) < 0 < np.max(den)):
        raise CalibrationError("Projection horizon crosses the supported area")
    train_errors = np.linalg.norm(project_points(h, src) - dst, axis=1)
    test_errors = np.linalg.norm(project_points(h, test_src) - test_dst, axis=1)
    numerical_pass = bool(np.max(test_errors) <= max_test_error_mm)
    content = {"homography": h.tolist(), "width": width, "height": height,
               "plane_height_mm": plane_height_mm, "support": support.tolist(), "provenance": provenance}
    digest = hashlib.sha256(json.dumps(content, sort_keys=True).encode("utf-8")).hexdigest()
    inferred_id = int(digest[:8], 16) or 1
    if calibration_id is None:
        calibration_id = inferred_id
    if isinstance(calibration_id, bool) or not isinstance(calibration_id, int) or not 1 <= calibration_id <= 0xFFFFFFFF:
        raise CalibrationError("calibration_id must be uint32 nonzero")
    return {"schema": "sorter.calibration.v1", "calibration_id": calibration_id,
            "calibration_confirmed": False, "provenance": provenance,
            "warning": "Offline fit only. Hardware accuracy and coordinate conventions are not verified.",
            "image": {"width": width, "height": height}, "plane_height_mm": plane_height_mm,
            "homography": h.tolist(), "support_pixel_polygon": support.tolist(),
            "support_world_polygon_mm": project_points(h, support).tolist(),
            "dlt_constraint_condition": condition, "calibration_content_sha256": digest,
            "training": error_metrics(train_errors), "independent_test": error_metrics(test_errors),
            "max_allowed_test_error_mm": max_test_error_mm, "numerical_test_passed": numerical_pass,
            "confirmation_permitted": provenance == "measured" and numerical_pass,
            "points": [dict(row, predicted_world_mm=project_points(h, [row["image"]])[0].tolist(),
                            error_mm=float(np.linalg.norm(project_points(h, [row["image"]])[0] - row["world"])))
                       for row in rows]}


def cpp_header(calibration):
    polygon = calibration["support_pixel_polygon"]
    matrix = [v for row in calibration["homography"] for v in row]
    provenance = calibration["provenance"]
    lines = ["#pragma once", "#include <stddef.h>", "#include <stdint.h>",
             '#include <Homography.h>', "", "// Generated by tools/calibrate.py. Review before installing.",
             f"// Provenance: {provenance}; NOT hardware validated.",
             "// CALIBRATION_CONFIRMED stays false even when numerical holdout passes.",
             "namespace sorter_calibration {",
             f"constexpr uint32_t CALIBRATION_ID = {calibration['calibration_id']}u;",
             "constexpr bool CALIBRATION_CONFIRMED = false;",
             f"constexpr uint16_t IMAGE_WIDTH = {calibration['image']['width']};",
             f"constexpr uint16_t IMAGE_HEIGHT = {calibration['image']['height']};",
             "constexpr double HOMOGRAPHY[9] = {" + ", ".join(format(v, ".17g") for v in matrix) + "};",
             f"constexpr size_t SUPPORT_COUNT = {len(polygon)};",
             "const sorter::Point2 SUPPORT[SUPPORT_COUNT] = {"]
    lines.extend("    {" + format(u, ".17g") + ", " + format(v, ".17g") + "}," for u, v in polygon)
    lines += ["};", "}  // namespace sorter_calibration", ""]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Planar calibration with independent holdout (no hardware access)")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="Output JSON")
    parser.add_argument("--header", type=Path, help="Optional generated C++ header")
    parser.add_argument("--provenance", choices=["measured", "synthetic", "unverified"], default="unverified")
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--plane-height-mm", type=float, required=True)
    parser.add_argument("--max-test-error-mm", type=float, default=2.0)
    parser.add_argument("--valid-rect", type=float, nargs=4, metavar=("U0", "V0", "U1", "V1"))
    parser.add_argument("--calibration-id", type=int)
    args = parser.parse_args(argv)
    try:
        paths = [args.csv.resolve(), args.output.resolve()]
        if args.header:
            paths.append(args.header.resolve())
        if len(set(paths)) != len(paths):
            raise CalibrationError("Input CSV, output JSON and C++ header must be different files")
        result = build_calibration(read_points(args.csv), width=args.width, height=args.height,
                                   plane_height_mm=args.plane_height_mm, provenance=args.provenance,
                                   valid_rect=args.valid_rect, max_test_error_mm=args.max_test_error_mm,
                                   calibration_id=args.calibration_id)
        result["source_csv_sha256"] = hashlib.sha256(args.csv.read_bytes()).hexdigest()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        if args.header:
            args.header.parent.mkdir(parents=True, exist_ok=True)
            args.header.write_text(cpp_header(result), encoding="utf-8")
    except (CalibrationError, OSError) as exc:
        parser.exit(2, f"Calibration failed: {exc}\n")
    print(json.dumps({"output": str(args.output), "provenance": result["provenance"],
                      "calibration_confirmed": False, "independent_test": result["independent_test"],
                      "numerical_test_passed": result["numerical_test_passed"]}, indent=2))
    return 0 if result["numerical_test_passed"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
