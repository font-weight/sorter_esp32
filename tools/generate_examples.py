"""Reproduce clearly synthetic calibration, camera and trial-log fixtures."""
import csv
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

try:
    from .calibrate import build_calibration, cpp_header, project_points
    from .protocol import Packet, Detection, Scene, encode_packet, encode_scene
    from .analyze_run import COLUMNS, analyze, read_log
except ImportError:
    from calibrate import build_calibration, cpp_header, project_points
    from protocol import Packet, Detection, Scene, encode_packet, encode_scene
    from analyze_run import COLUMNS, analyze, read_log


def generate(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    truth_h = np.array([[0.42, 0.025, -70.0], [0.015, 0.40, 55.0], [0.0006, -0.0004, 1.0]])
    train = [(u, v) for v in (25, 120, 215) for u in (30, 160, 290)]
    test = [(95, 72.5), (225, 72.5), (95, 167.5), (225, 167.5)]
    rows = []
    for split, points in (("train", train), ("test", test)):
        world = project_points(truth_h, points)
        for index, (pixel, xy) in enumerate(zip(points, world)):
            # Deterministic simulated measurement noise, not a sensor recording.
            noise = np.array([np.sin(index + 1), np.cos(index + 1)]) * (0.04 if split == "train" else 0.10)
            rows.append({"split": split, "image": list(pixel), "world": (xy + noise).tolist()})
    with (directory / "synthetic_calibration_points.csv").open("w", encoding="utf-8", newline="") as out:
        writer = csv.writer(out)
        writer.writerow(["split", "u", "v", "x_mm", "y_mm", "provenance"])
        for row in rows:
            writer.writerow([row["split"], *row["image"], *row["world"], "synthetic"])
    calibration = build_calibration(rows, provenance="synthetic", plane_height_mm=10)
    (directory / "synthetic_calibration.json").write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
    (directory / "SyntheticCalibration.h").write_text(cpp_header(calibration), encoding="utf-8")
    image = Image.new("RGB", (320, 240), (180, 180, 180))
    draw = ImageDraw.Draw(image)
    objects, ignored_objects = [], []
    for class_id, center, color in [(1, (90, 80), (255, 80, 50)), (1, (135, 170), (255, 80, 50)),
                                    (0, (185, 95), (35, 195, 45)), (3, (240, 165), (140, 140, 230))]:
        u, v = center
        draw.ellipse((u - 10, v - 10, u + 10, v + 10), fill=color)
        record = {"class_id": class_id, "center_uv": list(center), "radius_pixel": 10,
                  "world_xy_mm_synthetic": project_points(truth_h, [center])[0].tolist()}
        if class_id:
            objects.append(record)
        else:
            record["reason"] = "green distractor; must not be sorted"
            ignored_objects.append(record)
    image.save(directory / "synthetic_scene.png")
    image.save(directory / "synthetic_scene.ppm")
    rgb = np.asarray(image, dtype=np.uint16)
    values = ((rgb[:, :, 0] >> 3) << 11) | ((rgb[:, :, 1] >> 2) << 5) | (rgb[:, :, 2] >> 3)
    (directory / "synthetic_scene.rgb565").write_bytes(values.astype(">u2").tobytes())
    (directory / "synthetic_scene_truth.json").write_text(json.dumps({"provenance": "synthetic",
        "not_a_camera_capture": True, "width": 320, "height": 240, "format": "RGB565BE",
        "supported_classes": [1, 3], "true_homography": truth_h.tolist(),
        "objects": objects, "ignored_objects": ignored_objects}, indent=2) + "\n", encoding="utf-8")
    points_image = Image.new("RGB", (640, 480), "white")
    point_draw = ImageDraw.Draw(points_image)
    point_draw.text((15, 8), "SYNTHETIC - NOT MEASURED - 9 TRAIN / 4 TEST", fill="black")
    point_draw.rectangle((60, 50, 580, 430), outline=(120, 120, 120), width=2)
    for index, row in enumerate(rows):
        u, v = row["image"]
        x, y = round(u * 2), round(v * 2)
        color = (35, 90, 170) if row["split"] == "train" else (220, 110, 30)
        point_draw.ellipse((x-5, y-5, x+5, y+5), fill=color)
        point_draw.text((x+8, y-5), f"{row['split']} {index+1}", fill=color)
    points_image.save(directory / "synthetic_calibration_layout.png")
    rows_log = [
        ("synthetic_run", "p01", 1, 1, 1, "success", 14.0, "false", "SYNTHETIC fixture"),
        ("synthetic_run", "p02", 1, 3, "", "miss", 15.0, "false", "SYNTHETIC fixture"),
        ("synthetic_run", "p02", 2, 3, 3, "success", 16.0, "false", "SYNTHETIC retry"),
        ("synthetic_run", "p03", 1, 3, 1, "wrong_bin", 18.0, "false", "SYNTHETIC fixture"),
        ("synthetic_run", "p04", 1, 1, "", "drop", 12.0, "false", "SYNTHETIC fixture"),
        ("synthetic_run", "p05", 1, 3, 3, "success", 13.0, "false", "SYNTHETIC fixture"),
        ("synthetic_run", "p06", 1, 3, "", "timeout", 30.0, "false", "SYNTHETIC fixture"),
        ("synthetic_run", "p07", 1, 1, "", "aborted", 5.0, "true", "SYNTHETIC intervention"),
        ("synthetic_run", "p08", 1, 3, 3, "success", 15.0, "false", "SYNTHETIC fixture"),
    ]
    log_path = directory / "synthetic_run.csv"
    with log_path.open("w", encoding="utf-8", newline="") as out:
        writer = csv.writer(out)
        writer.writerow(COLUMNS)
        writer.writerows(rows_log)
    report = analyze(read_log(log_path), provenance="synthetic", expected_parts=8)
    (directory / "synthetic_run_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    vectors = [Packet("Q", 1, 1, "42"),
               Packet("D", 4294967295, 27, encode_scene(Scene(17, 42, (Detection(1, -250, 800, 321), Detection(3, 100, 1200, 298))))),
               Packet("E", 1, 1, "CONFLICT")]
    (directory / "protocol_vectors.json").write_text(json.dumps({"crc_algorithm": "CRC16-CCITT-FALSE",
        "check_123456789_hex": "29B1", "vectors": [{"type": p.type, "session": p.session, "seq": p.seq,
          "payload": p.payload, "frame": encode_packet(p).decode("ascii")} for p in vectors]}, indent=2) + "\n", encoding="utf-8")
    (directory / "calibration_points_template.csv").write_text("split,u,v,x_mm,y_mm\n", encoding="utf-8")
    (directory / "run_log_template.csv").write_text(",".join(COLUMNS) + "\n", encoding="utf-8")
    print(f"Synthetic examples written to {directory}; no hardware was accessed.")


if __name__ == "__main__":
    generate(Path(__file__).resolve().parents[1] / "examples")
