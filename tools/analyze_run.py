"""Analyze all recorded attempts without silently discarding failures."""
import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

try:
    from .protocol import SUPPORTED_CLASSES
except ImportError:
    from protocol import SUPPORTED_CLASSES


OUTCOMES = {"success", "wrong_bin", "miss", "pick_fail", "drop", "timeout", "aborted"}
COLUMNS = ("run_id", "part_id", "attempt_index", "expected_class", "placed_class", "outcome",
           "cycle_s", "intervention", "notes")


class RunLogError(ValueError):
    pass


def read_log(path):
    rows = []
    with Path(path).open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not set(COLUMNS).issubset(reader.fieldnames or []):
            raise RunLogError("CSV columns required: " + ",".join(COLUMNS))
        for line, row in enumerate(reader, 2):
            try:
                if not row["run_id"] or not row["part_id"]:
                    raise ValueError("Empty run_id or part_id")
                attempt = int(row["attempt_index"])
                expected = int(row["expected_class"])
                placed = None if row["placed_class"] == "" else int(row["placed_class"])
                duration = float(row["cycle_s"])
                intervention_raw = row["intervention"].strip().lower()
                if attempt < 1 or expected not in SUPPORTED_CLASSES or placed not in (None, *SUPPORTED_CLASSES):
                    raise ValueError("Invalid attempt or class")
                if not math.isfinite(duration) or duration < 0:
                    raise ValueError("Invalid cycle_s")
                if intervention_raw not in ("true", "false", "1", "0"):
                    raise ValueError("intervention must be true/false or 1/0")
                intervention = intervention_raw in ("true", "1")
                outcome = row["outcome"]
                if outcome not in OUTCOMES:
                    raise ValueError("Unknown outcome")
                if outcome == "success" and (placed != expected or intervention):
                    raise ValueError("success requires correct class and no intervention")
                if outcome == "wrong_bin" and (placed is None or placed == expected):
                    raise ValueError("wrong_bin requires a different actual class")
            except (ValueError, TypeError) as exc:
                raise RunLogError(f"CSV line {line}: {exc}") from exc
            rows.append({"run_id": row["run_id"], "part_id": row["part_id"], "attempt_index": attempt,
                         "expected_class": expected, "placed_class": placed, "outcome": outcome,
                         "cycle_s": duration, "intervention": intervention, "notes": row["notes"]})
    return rows


def percentile(values, percent):
    ordered = sorted(values)
    if not ordered:
        return None
    at = (len(ordered) - 1) * percent / 100
    low = math.floor(at)
    high = math.ceil(at)
    return ordered[low] + (ordered[high] - ordered[low]) * (at - low)


def duration_stats(values):
    if not values:
        return {"count": 0, "mean_s": None, "median_s": None, "p95_s": None, "max_s": None}
    return {"count": len(values), "mean_s": sum(values) / len(values),
            "median_s": percentile(values, 50), "p95_s": percentile(values, 95), "max_s": max(values)}


def analyze(rows, provenance="unverified", expected_parts=None):
    if not rows:
        raise RunLogError("Empty log is not a completed trial")
    grouped = {}
    for row in rows:
        key = (row["run_id"], row["part_id"])
        grouped.setdefault(key, []).append(row)
    for key, attempts in grouped.items():
        ordered = sorted(attempts, key=lambda r: r["attempt_index"])
        if [r["attempt_index"] for r in ordered] != list(range(1, len(ordered) + 1)):
            raise RunLogError(f"{key}: attempts must begin at 1 and be unique, contiguous")
        if len({r["expected_class"] for r in ordered}) != 1:
            raise RunLogError(f"{key}: expected class changed between attempts")
        if any(r["outcome"] == "success" for r in ordered[:-1]):
            raise RunLogError(f"{key}: an already successful trial cannot be retried")
    if expected_parts is not None and expected_parts < len(grouped):
        raise RunLogError("expected_parts is smaller than the number of logged trials")
    first = [min(value, key=lambda r: r["attempt_index"]) for value in grouped.values()]
    first_success = sum(r["outcome"] == "success" for r in first)
    success = sum(r["outcome"] == "success" for r in rows)
    denominator = expected_parts if expected_parts is not None else len(grouped)
    return {"schema": "sorter.run_report.v1", "provenance": provenance,
            "warning": "Synthetic logs are not evidence of physical robot performance." if provenance == "synthetic"
                       else "Only logged or explicitly counted planned trials are represented; verify log completeness.",
            "logged_trials": len(grouped), "planned_trials": expected_parts,
            "unlogged_planned_trials": 0 if expected_parts is None else expected_parts - len(grouped),
            "primary_denominator_trials": denominator,
            "first_attempt_successes": first_success,
            "first_attempt_success_rate": first_success / denominator,
            "eventual_successes": success, "eventual_success_rate": success / denominator,
            "all_attempts": len(rows), "failed_attempts": len(rows) - success,
            "all_attempt_success_rate": success / len(rows),
            "interventions": sum(r["intervention"] for r in rows),
            "outcomes": dict(sorted(Counter(r["outcome"] for r in rows).items())),
            "all_attempt_durations": duration_stats([r["cycle_s"] for r in rows]),
            "successful_attempt_durations": duration_stats([r["cycle_s"] for r in rows if r["outcome"] == "success"]),
            "first_attempt_durations": duration_stats([r["cycle_s"] for r in first]),
            "percentile_method": "linear interpolation at (n-1)*p; seconds",
            "interpretation": "Primary rate counts correct placement without help on the first attempt; retries do not erase failures."}


def main(argv=None):
    parser = argparse.ArgumentParser(description="Analyze sorter attempt log, keeping failures in denominators")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--provenance", choices=("synthetic", "measured", "unverified"), default="unverified")
    parser.add_argument("--expected-parts", type=int, help="Number of planned trials; unlogged trials remain failures")
    args = parser.parse_args(argv)
    try:
        if args.expected_parts is not None and args.expected_parts < 1:
            raise RunLogError("expected_parts must be positive")
        result = analyze(read_log(args.csv), args.provenance, args.expected_parts)
        text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text, encoding="utf-8")
        print(text, end="")
    except (RunLogError, OSError) as exc:
        parser.exit(2, f"Log analysis failed: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
