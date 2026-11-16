"""What a submission is: a folder with controller.py (plus any modules and assets/).

A single .py file also works (the example controllers are submitted that way); it is
treated as a folder that holds only that file.
"""

from dataclasses import dataclass
import hashlib
import importlib.util
from pathlib import Path
import sys

ENTRY = "controller.py"
MAX_SUBMISSION_BYTES = 50 * 1024 * 1024
IGNORED_PARTS = {"__pycache__", ".DS_Store", ".ipynb_checkpoints", ".git"}


@dataclass(frozen=True)
class Submission:
    root: Path      # folder that is put on sys.path and used as the working directory
    entry: Path     # file that defines Controller

    @property
    def files(self) -> list[Path]:
        if self.entry.name != ENTRY or self.entry.parent != self.root:
            return [self.entry]
        return sorted(p for p in self.root.rglob("*") if p.is_file() and not IGNORED_PARTS & set(p.parts))

    def sha256(self) -> str:
        digest = hashlib.sha256()
        for path in self.files:
            digest.update(path.relative_to(self.root).as_posix().encode() + b"\0")
            digest.update(hashlib.sha256(path.read_bytes()).digest())
        return digest.hexdigest()

    def size_bytes(self) -> int:
        return sum(path.stat().st_size for path in self.files)


def resolve(path: str | Path) -> Submission:
    path = Path(path).resolve()
    if path.is_dir():
        entry = path / ENTRY
        if not entry.is_file():
            raise FileNotFoundError(f"{path} has no {ENTRY}")
        return Submission(path, entry)
    if path.is_file() and path.suffix == ".py":
        return Submission(path.parent, path)
    raise FileNotFoundError(f"Submission must be a folder with {ENTRY} or a .py file: {path}")


def layout_problems(submission: Submission) -> list[str]:
    problems = []
    size = submission.size_bytes()
    if size > MAX_SUBMISSION_BYTES:
        problems.append(f"submission is {size / 2**20:.1f} MB, the limit is {MAX_SUBMISSION_BYTES / 2**20:.0f} MB")
    return problems


def find_repo_file(name: str) -> Path | None:
    """A file of the competition repository (requirements.txt, examples/...), searched
    from the working directory upwards, then next to the installed package."""
    starts = [Path.cwd(), Path(__file__).resolve().parents[2]]
    for start in starts:
        for folder in (start, *start.parents):
            if (folder / name).exists() and (folder / "pyproject.toml").is_file():
                return folder / name
    return None


def load_controller(submission: Submission):
    """Import the entry file and construct Controller (used by the worker and by --debug)."""
    root = str(submission.root)
    if root not in sys.path:
        sys.path.insert(0, root)
    spec = importlib.util.spec_from_file_location("controller", submission.entry)
    module = importlib.util.module_from_spec(spec)
    sys.modules["controller"] = module
    spec.loader.exec_module(module)
    controller = module.Controller()
    if not callable(getattr(controller, "reset", None)) or not callable(getattr(controller, "act", None)):
        raise TypeError("Controller must implement reset(config) and act(observation)")
    return controller
