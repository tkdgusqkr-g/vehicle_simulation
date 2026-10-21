import pathlib
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "python"))

SCENARIOS = REPO_ROOT / "templates" / "projects" / "example_fs_ev" / "scenarios"


@pytest.fixture
def scenario():
    """Path of an example scenario by name."""
    return lambda name: str(SCENARIOS / f"{name}.yaml")
