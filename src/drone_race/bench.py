"""Estimate how fast a submission's act() will be on the official machine.

    drone-race bench                       # calibrate this computer, then time your submission/
    drone-race bench --write-reference     # TAs, once, on the official Linux server

Three fixed workloads in ``bench_workloads/`` are the yardstick: controllers that hover
and do a fixed amount of NumPy, SciPy and plain-Python work in every act(). Their act()
times were measured on the official machine and are stored in
``data/reference_machine.json``. Running them here gives this computer's speed
relative to that machine, which then scales the timings of your own controller.
It is an estimate (CPU architecture and JIT compilers shift it by tens of percent);
practice runs on the official machine are the final word.

The measured ratio is saved to outputs/calibration.json. From then on ``drone-race
run`` shrinks or stretches the 20 ms budget on this computer by that ratio, so a
controller that is late here would also be late on the scoring server.
"""

from dataclasses import replace
import datetime
import json
from pathlib import Path
import platform
import os

import numpy as np

from .env import Scenario, run_episode
from .race import load_race
from . import submission as submissions

REFERENCE = Path(__file__).with_name("data") / "reference_machine.json"


def calibration_path() -> Path:
    project = submissions.find_repo_file("pyproject.toml")
    return (project.parent if project else Path.cwd()) / "outputs" / "calibration.json"


def load_calibration() -> dict | None:
    """This computer's saved calibration, if it was made against the current reference."""
    try:
        saved = json.loads(calibration_path().read_text())
        reference = json.loads(REFERENCE.read_text())
    except (OSError, ValueError):
        return None
    if saved.get("reference_median_act_s") != reference["median_act_s"]:
        return None          # the reference machine was re-measured since: calibrate again
    return saved
# Fixed CPU workloads (hover + a fixed amount of NumPy, SciPy and plain-Python work per act()).
WORKLOADS = {name: Path(__file__).with_name("bench_workloads") / f"{name}.py"
             for name in ("numpy_rollout", "scipy_linalg", "python_loop")}
UNLIMITED = 1e6     # budget scale that turns deadline enforcement off while measuring


def act_times(submission, config, seconds: float) -> np.ndarray:
    log = run_episode(submission, config, Scenario(max_time_s=seconds), budget_scale=UNLIMITED)
    times = np.array([s["act_s"] for s in log["samples"] if s["act_s"] is not None])
    return times[5:] if len(times) > 20 else times     # skip warm-up calls


def machine() -> dict:
    return {"platform": platform.platform(), "processor": platform.processor() or platform.machine(),
            "python": platform.python_version(), "cpus": os.cpu_count()}


def measure_workloads(config, seconds: float, progress=print) -> dict[str, float]:
    medians = {}
    for name, path in WORKLOADS.items():
        medians[name] = float(np.median(act_times(path, config, seconds)))
        progress(f"  {name:<13} median act {1e3 * medians[name]:6.2f} ms")
    return medians


def bench(submission="submission", race_path=None, *, seconds=10.0, write_reference=False, progress=print):
    race_path = race_path or submissions.find_repo_file("config/race.json")
    _, config, _ = load_race(race_path, laps=1)
    reference_seconds = 4.0
    progress(f"this machine: {machine()['processor']}, {machine()['platform']}")
    progress("reference workloads (single thread):")
    local = measure_workloads(config, reference_seconds, progress)
    if write_reference:
        REFERENCE.parent.mkdir(exist_ok=True)
        REFERENCE.write_text(json.dumps({
            "placeholder": False, "measured": datetime.date.today().isoformat(), "machine": machine(),
            "track": config.track.name, "seconds": reference_seconds, "median_act_s": local,
        }, indent=2) + "\n")
        progress(f"wrote {REFERENCE}")
        return None
    reference = json.loads(REFERENCE.read_text())
    if set(reference["median_act_s"]) != set(WORKLOADS):
        raise RuntimeError(f"{REFERENCE.name} was measured with other workloads "
                           f"({', '.join(reference['median_act_s'])}); TAs must re-run `drone-race bench --write-reference`")
    ratio = float(np.exp(np.mean([np.log(reference["median_act_s"][k] / local[k]) for k in WORKLOADS])))
    path = calibration_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({
        "measured": datetime.datetime.now().isoformat(timespec="minutes"), "machine": machine(),
        "ratio": ratio, "budget_scale": 1 / ratio, "local_median_act_s": local,
        "reference_median_act_s": reference["median_act_s"], "reference_placeholder": reference.get("placeholder", False),
    }, indent=2) + "\n")
    progress(f"official machine: {reference['machine']['processor']} ({reference['measured']})"
             + ("  [PLACEHOLDER -- not measured on the official server yet]" if reference.get("placeholder") else ""))
    progress(f"speed ratio: the official machine takes {ratio:.2f}x as long as this one")

    times = act_times(submission, config, seconds)
    budget = config.act_budget_s
    estimate = times * ratio
    p50, p99, worst = (float(np.percentile(estimate, q)) for q in (50, 99, 100))
    late = float(np.mean(estimate > budget))
    progress(f"\nyour controller, {len(times)} act() calls, estimated on the official machine:")
    progress(f"  median {1e3 * p50:6.2f} ms   p99 {1e3 * p99:6.2f} ms   max {1e3 * worst:6.2f} ms"
             f"   budget {1e3 * budget:.0f} ms")
    progress(f"  late calls: {100 * late:.1f}%   (a late act() is dropped and the previous action held)")
    margin = 1 - p99 / budget
    verdict = "OK" if margin >= 0.3 else "TIGHT (aim for >= 30% margin)" if margin >= 0 else "TOO SLOW"
    progress(f"  margin at p99: {100 * margin:.0f}%  -> {verdict}")
    progress(f"\nSaved {path}: `drone-race run` now enforces a {1e3 * budget / ratio:.1f} ms budget on this computer,"
             f"\nthe equivalent of {1e3 * budget:.0f} ms on the scoring server.")
    return {"ratio": ratio, "p50_s": p50, "p99_s": p99, "max_s": worst, "late_fraction": late}
