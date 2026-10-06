"""Historical entry points must reject execution without touching hardware/data."""
from pathlib import Path
import subprocess
import sys

import pytest

ARCHIVE = Path(__file__).resolve().parents[1]/'scripts/archive'


@pytest.mark.parametrize('name', [
    'placement_execute.py', 'placement_release.py', 'placement_ik.py',
    'correct_grasp_opening_axis.py', 'search_grasp_joint_routes.py',
])
def test_completed_task_entry_is_retired(name, tmp_path):
    result = subprocess.run([sys.executable, str(ARCHIVE/name), '--execute'],
                            cwd=tmp_path, capture_output=True, text=True, timeout=15)
    assert result.returncode != 0
    assert 'cannot be replayed' in result.stderr
    assert list(tmp_path.iterdir()) == []
