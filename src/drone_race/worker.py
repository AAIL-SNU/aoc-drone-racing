"""Run a submission in its own process and talk to it over JSON lines.

Host side: ``WorkerController`` starts ``python -m drone_race.worker`` (optionally
inside ``docker run ...``) and exchanges one message per reset()/act() call.
``InlineController`` loads the submission into the simulator's own process instead;
it exists for debugging (breakpoints, print-heavy work) and does not enforce timing.

Compute time is the worker's CPU time (all of its threads), not wall-clock time:
being preempted by the OS never makes a call late, and extra threads never help.
Each call is charged everything since the previous call ended, so CPU used between
calls counts too, and importing the submission counts towards reset().

Worker side: ``main()``. The protocol uses a private copy of stdout, and fd 1 is
pointed at stderr, so ``print`` in student code can never corrupt a message.
"""

from contextlib import contextmanager
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import traceback

# Every common numerical runtime limited to one thread. Set in the worker's
# environment before Python starts, so it applies before numpy is first imported.
THREAD_ENV = {
    "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
    "VECLIB_MAXIMUM_THREADS": "1", "NUMEXPR_NUM_THREADS": "1", "NUMBA_NUM_THREADS": "1",
    "RAYON_NUM_THREADS": "1", "BLIS_NUM_THREADS": "1",
    "XLA_FLAGS": "--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1",
    "PYTHONHASHSEED": "0", "PYTHONDONTWRITEBYTECODE": "1", "PYTHONUNBUFFERED": "1",
}
STARTUP_TIMEOUT_S = 120.0   # wall-clock safety net for a hung import; its CPU is charged to reset()


class ControllerError(Exception):
    status = "controller_error"


class ControllerTimeout(ControllerError):
    status = "controller_timeout"


def _describe(exc: BaseException) -> str:
    return f"{type(exc).__name__}: {exc}"


# ---------------------------------------------------------------------------- host side
class WorkerController:
    enforces_timing = True

    def __init__(self, submission, *, command_prefix=None, mount_root=None, cpu=None, stderr=None):
        """``command_prefix`` (e.g. a ``docker run ...`` list) runs the worker elsewhere;
        ``mount_root`` is where the submission folder appears inside that environment."""
        root = Path(mount_root) if mount_root else submission.root
        entry = root / submission.entry.relative_to(submission.root)
        if command_prefix:
            command = [*command_prefix, "python", "-m", "drone_race.worker", entry.as_posix(), root.as_posix()]
        else:
            command = [sys.executable, "-m", "drone_race.worker", str(entry), str(root)]
        env = {**os.environ, **THREAD_ENV}
        if cpu is not None:
            env["DRONE_RACE_CPU"] = str(cpu)
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=stderr,
                                        env=env, text=True, bufsize=1)
        self.replies = queue.Queue()
        threading.Thread(target=self._read, daemon=True).start()
        self.hello = self._receive(STARTUP_TIMEOUT_S, "import")

    def _read(self):
        for line in self.process.stdout:
            self.replies.put(line)
        self.replies.put(None)

    def _receive(self, timeout, what):
        try:
            line = self.replies.get(timeout=timeout)
        except queue.Empty:
            self.kill()
            raise ControllerTimeout(f"{what}: no reply within {timeout:.3g} s") from None
        if line is None:
            code = self.process.wait()
            raise ControllerError(f"{what}: controller process exited (code {code}); see its stderr")
        reply = json.loads(line)
        if not reply.get("ok"):
            raise ControllerError(f"{what}: {reply.get('error')}")
        return reply

    def _call(self, message, timeout, what):
        try:
            self.process.stdin.write(json.dumps(message, allow_nan=True) + "\n")
            self.process.stdin.flush()
        except (BrokenPipeError, OSError):
            raise ControllerError(f"{what}: controller process is gone; see its stderr") from None
        return self._receive(timeout, what)

    def reset(self, config, timeout):
        from .protocol import config_to_dict
        return self._call({"type": "reset", "config": config_to_dict(config)}, timeout, "reset")

    def act(self, observation, timeout):
        from .protocol import observation_to_dict
        return self._call({"type": "act", "observation": observation_to_dict(observation)}, timeout, "act")

    def kill(self):
        if self.process.poll() is None:
            self.process.kill()
            self.process.wait()

    def close(self):
        if self.process.poll() is None:
            try:
                self.process.stdin.write(json.dumps({"type": "close"}) + "\n")
                self.process.stdin.close()
                self.process.wait(timeout=5)
            except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
                pass
        self.kill()


class InlineController:
    """Same interface as WorkerController, in the simulator's process; never times out."""
    enforces_timing = False

    def __init__(self, submission):
        from .submission import load_controller
        self.submission = submission
        with self._inside():
            try:
                self.controller = load_controller(submission)
            except Exception as exc:
                traceback.print_exc()
                raise ControllerError(f"import: {_describe(exc)}") from exc
        self.hello = {"ok": True, "modules": third_party_modules(submission.root)}

    @contextmanager
    def _inside(self):
        previous = os.getcwd()
        os.chdir(self.submission.root)
        try:
            yield
        finally:
            os.chdir(previous)

    def reset(self, config, timeout=None):
        from .protocol import config_from_dict, config_to_dict
        config = config_from_dict(config_to_dict(config))
        with self._inside():
            t0 = time.process_time()
            try:
                self.controller.reset(config)
            except Exception as exc:
                traceback.print_exc()
                raise ControllerError(f"reset: {_describe(exc)}") from exc
            elapsed = time.process_time() - t0
        return {"ok": True, "elapsed": elapsed, "modules": third_party_modules(self.submission.root)}

    def act(self, observation, timeout=None):
        from .protocol import action_to_wire, observation_from_dict, observation_to_dict
        observation = observation_from_dict(observation_to_dict(observation))
        with self._inside():
            t0 = time.process_time()
            try:
                raw = self.controller.act(observation)
            except Exception as exc:
                traceback.print_exc()
                raise ControllerError(f"act: {_describe(exc)}") from exc
            elapsed = time.process_time() - t0
        action, invalid = action_to_wire(raw)
        return {"ok": True, "elapsed": elapsed, "action": action, "invalid": invalid}

    def close(self):
        pass


def third_party_modules(root) -> list[str]:
    """Distributions (pip names) of every non-stdlib top-level module imported so far."""
    import importlib.metadata
    root = Path(root).resolve()
    by_module = importlib.metadata.packages_distributions()
    found = set()
    for name, module in list(sys.modules.items()):
        top = name.partition(".")[0]
        if "." in name or top in sys.stdlib_module_names or top.startswith("_") or top in {"drone_race", "controller"}:
            continue
        origin = getattr(module, "__file__", None)
        if origin is None:
            continue
        try:
            if Path(origin).resolve().is_relative_to(root):
                continue          # the submission's own modules
        except OSError:
            pass
        found.update(by_module.get(top, [top]))
    return sorted(found)


# ---------------------------------------------------------------------------- worker side
def main(argv=None):
    entry, root = (argv or sys.argv[1:])[:2]
    protocol = os.fdopen(os.dup(1), "w", buffering=1)
    os.dup2(2, 1)                                   # student prints go to stderr
    cpu = os.environ.get("DRONE_RACE_CPU")
    if cpu is not None and hasattr(os, "sched_setaffinity"):
        os.sched_setaffinity(0, {int(cpu)})

    def send(message):
        protocol.write(json.dumps(message, allow_nan=True) + "\n")
        protocol.flush()

    from .protocol import action_to_wire, config_from_dict, observation_from_dict
    from .submission import Submission, load_controller

    submission = Submission(Path(root).resolve(), Path(entry).resolve())
    os.chdir(submission.root)
    # Each call is charged all CPU the process used since the previous call ended, not just
    # the CPU inside reset()/act(): importing the submission counts towards reset(), and work
    # done by background threads between calls (while the simulator steps the physics) lands
    # on the next call instead of being free.
    last = time.process_time()
    try:
        controller = load_controller(submission)
    except BaseException as exc:
        traceback.print_exc()
        send({"ok": False, "error": _describe(exc)})
        return 1
    own = time.thread_time()
    modules = third_party_modules(submission.root)
    last += time.thread_time() - own               # the simulator's bookkeeping, not the controller's
    send({"ok": True, "python": sys.version.split()[0], "modules": modules})

    for line in sys.stdin:
        message = json.loads(line)
        kind = message["type"]
        if kind == "close":
            break
        try:
            if kind == "reset":
                config = config_from_dict(message["config"])
                controller.reset(config)
                now = time.process_time()
                elapsed = now - last
                # Listing the imports is the simulator's work, not the controller's: shift the
                # baseline by this thread's share, so other threads' CPU meanwhile still counts.
                own = time.thread_time()
                modules = third_party_modules(submission.root)
                last = now + time.thread_time() - own
                send({"ok": True, "elapsed": elapsed, "modules": modules})
            elif kind == "act":
                observation = observation_from_dict(message["observation"])
                raw = controller.act(observation)
                now = time.process_time()
                elapsed, last = now - last, now
                action, invalid = action_to_wire(raw)
                send({"ok": True, "elapsed": elapsed, "action": action, "invalid": invalid})
            else:
                send({"ok": False, "error": f"unknown message {kind!r}"})
        except BaseException as exc:
            traceback.print_exc()
            send({"ok": False, "error": _describe(exc)})
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
