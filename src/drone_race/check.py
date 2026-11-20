"""Pre-submission self-check: does this submission load and fly at all?

    drone-race check submission/

Scoring other people's submissions is the organiser's job and lives in the
admin repository, not here.
"""

from pathlib import Path
import re

from .env import run_episode
from .race import load_race
from . import submission as submissions


# ---------------------------------------------------------------------------- self-check
def _normalize(name: str) -> str:
    return re.sub(r"[-_.]+", "-", name).lower()


def allowed_packages(requirements: Path) -> dict[str, str]:
    """Pinned distributions of the official environment: {normalized name: version}."""
    pins = {}
    for line in requirements.read_text().splitlines():
        line = line.split("#")[0].strip()
        if "==" in line:
            dist, version = line.split("==", 1)
            pins[_normalize(dist)] = version.strip()
    return pins


def check(submission, race_path=None, *, seconds=3.0, progress=print) -> bool:
    """Layout, packages and a short worker run. Returns True when nothing blocks submission."""
    import importlib.metadata
    from .api import PublicConfig
    from .env import Scenario
    from .track import demo_track

    ok = True
    sub = submissions.resolve(submission)
    progress(f"submission   {sub.root}  ({sub.size_bytes() / 2**20:.2f} MB, sha256 {sub.sha256()[:12]})")
    for problem in submissions.layout_problems(sub):
        progress(f"  ✗ {problem}")
        ok = False

    requirements = submissions.find_repo_file("requirements.txt")
    pins = allowed_packages(requirements) if requirements else {}
    if requirements:
        drift = []
        for dist, version in pins.items():
            try:
                installed = importlib.metadata.version(dist)
            except importlib.metadata.PackageNotFoundError:
                installed = None
            if installed != version:
                drift.append(f"{dist} {installed or 'missing'} (official {version})")
        progress("environment  " + ("matches requirements.txt" if not drift else "differs from the official pins:"))
        for line in drift:
            progress(f"  ! {line}")
    else:
        progress("environment  requirements.txt not found; package check skipped")

    if race_path:
        _, config, scenario = load_race(race_path, laps=1)
    else:
        config, scenario = PublicConfig(demo_track()), Scenario()
    from dataclasses import replace
    log = run_episode(sub.root if sub.entry.name == submissions.ENTRY else sub.entry, config,
                      replace(scenario, max_time_s=seconds))
    s = log["summary"]
    crashed = s["status"] in ("controller_error", "controller_timeout", "invalid_action")
    outcome = {"timeout": "ran without errors", "finished": "finished the lap"}.get(s["status"], s["status"])
    progress(f"short run    {seconds:g} s on {config.track.name}: {'✗ ' if crashed else ''}{outcome}"
             + (f" -- {s['error']}" if s["error"] else ""))
    progress(f"  reset {s['controller_reset_s']:.2f} s (budget {config.reset_budget_s:.0f} s), "
             f"act mean {1e3 * s['controller_mean_act_s']:.1f} ms, p99 {1e3 * s['controller_p99_act_s']:.1f} ms "
             f"(budget {1e3 * config.act_budget_s:.0f} ms on the official machine; run `drone-race bench`)")
    ok &= not crashed
    used = log["provenance"]["third_party_modules"]
    if pins:
        outside = [m for m in used if _normalize(m) not in pins and _normalize(m) != "drone-race"]
        progress("packages     " + (", ".join(used) or "none beyond the standard library"))
        if outside:
            progress(f"  ✗ not in the official environment: {', '.join(outside)}")
            ok = False
    progress("result       " + ("OK" if ok else "FIX THE ITEMS MARKED ✗"))
    return ok
