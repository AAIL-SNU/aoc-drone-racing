"""Official scoring of many submissions, and the pre-submission self-check.

    drone-race evaluate teams/ --race config/races/official.json --seed 1234 --cpu 3
    drone-race check submission/

``evaluate`` runs one team after another (never in parallel, so timing is comparable)
and writes outputs/evaluation/logs/<team>.json, .../stderr/<team>.txt and
outputs/evaluation/leaderboard.json, the input for the dashboard.
"""

import json
from pathlib import Path
import re

from .env import run_episode, save_log
from .race import load_race
from . import submission as submissions
from .worker import THREAD_ENV


def rank(summaries: dict[str, dict]) -> list[dict]:
    """Finished runs by score; then unfinished ones by progress, then by time survived."""
    rows = []
    for team, s in summaries.items():
        rows.append({
            "team": team, "status": s["status"], "completed": s["completed"],
            "score_s": s["score_s"] if s["completed"] else None, "flight_time_s": s["flight_time_s"],
            "penalty_s": s["penalty_s"], "laps_completed": s["laps_completed"], "laps_total": s["laps_total"],
            "lap_times_s": s["lap_times_s"], "fastest_lap_s": s["fastest_lap_s"],
            "gates_resolved": s["gates_resolved"], "gates_missed": s["gates_missed"], "infractions": s["infractions"],
            "deadline_misses": s["deadline_misses"], "act_p99_ms": 1e3 * s["controller_p99_act_s"],
            "error": s["error"],
        })
    rows.sort(key=lambda r: (0, r["score_s"], 0) if r["completed"] else (1, -r["gates_resolved"], -r["flight_time_s"]))
    for place, row in enumerate(rows, 1):
        row["rank"] = place
    return rows


def docker_prefix(image: str, team_dir: Path, cpu=None, memory="4g") -> list[str]:
    command = ["docker", "run", "--rm", "-i", "--network", "none", "--read-only", "--tmpfs", "/tmp",
               "--cpus", "1", "--memory", memory, "--memory-swap", memory, "--pids-limit", "256"]
    if cpu is not None:
        command += ["--cpuset-cpus", str(cpu)]
    for key, value in THREAD_ENV.items():
        command += ["-e", f"{key}={value}"]
    return command + ["-v", f"{team_dir.resolve()}:/submission:ro", image]


def evaluate(submissions_dir, race_path, out_dir, *, seed=None, laps=None, cpu=None, docker_image=None,
             teams=None, progress=print) -> list[dict]:
    submissions_dir, out_dir = Path(submissions_dir), Path(out_dir)
    name, config, scenario = load_race(race_path, laps=laps, seed=seed)
    team_dirs = sorted(p for p in submissions_dir.iterdir() if p.is_dir() and (p / submissions.ENTRY).is_file())
    if teams:
        team_dirs = [p for p in team_dirs if p.name in set(teams)]
    if not docker_image:
        progress("warning: running without --docker; submissions are isolated by process only (not a sandbox)")
    (out_dir / "logs").mkdir(parents=True, exist_ok=True)
    (out_dir / "stderr").mkdir(parents=True, exist_ok=True)
    summaries, digests = {}, {}
    for team_dir in team_dirs:
        team = team_dir.name
        problems = submissions.layout_problems(submissions.resolve(team_dir))
        progress(f"[{team}] running {name} ({config.laps} laps, seed {scenario.seed})"
                 + (f" -- {'; '.join(problems)}" if problems else ""))
        with open(out_dir / "stderr" / f"{team}.txt", "w") as stderr:
            log = run_episode(team_dir, config, scenario, cpu=None if docker_image else cpu, stderr=stderr,
                              command_prefix=docker_prefix(docker_image, team_dir, cpu) if docker_image else None,
                              mount_root="/submission" if docker_image else None,
                              isolation="docker" if docker_image else "process")
        if problems:
            log["summary"].update(status="disqualified", completed=False, error="; ".join(problems))
        log["race"] = {"name": name, "team": team}
        save_log(log, out_dir / "logs" / f"{team}.json")
        summaries[team], digests[team] = log["summary"], log["provenance"]["submission_sha256"]
        s = log["summary"]
        progress(f"[{team}] {s['status']}  score {s['score_s']:.2f} s  laps {s['laps_completed']}/{s['laps_total']}"
                 f"  missed {s['gates_missed']}  deadline misses {s['deadline_misses']}")
    rows = rank(summaries)
    for row in rows:
        row["submission_sha256"] = digests[row["team"]]
    board = {"race": name, "laps": config.laps, "seed": scenario.seed, "track": config.track.name, "results": rows}
    (out_dir / "leaderboard.json").write_text(json.dumps(board, indent=2) + "\n")
    return rows


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
