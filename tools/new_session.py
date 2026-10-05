"""Create empty measurement templates without overwriting an existing session."""
import argparse
import json
from pathlib import Path

try:
    from .analyze_run import COLUMNS
except ImportError:
    from analyze_run import COLUMNS


def main(argv=None):
    parser = argparse.ArgumentParser(description="Create a NEW empty measurement folder; existing paths are refused")
    parser.add_argument("directory", type=Path)
    parser.add_argument("--planned-trials", type=int)
    args = parser.parse_args(argv)
    if args.planned_trials is not None and args.planned_trials < 1:
        parser.error("planned-trials must be positive")
    try:
        args.directory.mkdir(parents=True, exist_ok=False)
        (args.directory / "calibration_points.csv").write_text("split,u,v,x_mm,y_mm\n", encoding="utf-8")
        (args.directory / "run_log.csv").write_text(",".join(COLUMNS) + "\n", encoding="utf-8")
        (args.directory / "session.json").write_text(json.dumps({
            "status": "not_measured", "planned_trials": args.planned_trials,
            "camera_board": None, "sensor": None, "firmware_revision": None,
            "image_width_measured": None, "image_height_measured": None,
            "plane_height_mm_measured": None, "lighting_description": None,
            "coordinate_origin_and_axes": None, "operator": None, "date": None,
            "note": "Fill from the physical setup. Null means unknown, not zero."}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        (args.directory / "README.md").write_text(
            "# Новый сеанс измерений\n\n"
            "Папка создана пустой. Камера и робот не запускались; измерений пока нет.\n\n"
            "1. Заполните сведения о стенде в session.json. Неизвестные поля оставьте null.\n"
            "2. Сохраните исходный кадр камеры в эту папку и откройте его в tools/calibration_picker.html.\n"
            "3. Внесите реальные соответствия в calibration_points.csv либо сохраните CSV из страницы разметки.\n"
            "4. Каждую запланированную попытку запишите в run_log.csv, включая промах, отказ и вмешательство.\n"
            "5. Храните отчёты расчёта отдельно от исходных измерений.\n\n"
            "Шаблоны не содержат синтетических точек или успешных переносов.\n", encoding="utf-8")
    except FileExistsError:
        parser.exit(2, f"Session already exists; nothing overwritten: {args.directory}\n")
    except OSError as exc:
        parser.exit(2, f"Could not finish session creation: {exc}. Check the new folder; no existing files were replaced.\n")
    print(f"Empty session created: {args.directory}. No measurements performed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
