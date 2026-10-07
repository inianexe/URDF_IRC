from pathlib import Path
import subprocess
import sys


DESCRIPTION = Path(__file__).resolve().parents[2] / 'rover_description'


def test_rulebook_hard_limits_against_expanded_model():
    result = subprocess.run(
        [sys.executable, str(DESCRIPTION/'scripts/check_irc_2026_compliance.py')],
        text=True, capture_output=True, timeout=90)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'base+arm: mass=65.000 kg' in result.stdout
    assert 'IRC 2026 modeled compliance check: PASS' in result.stdout
