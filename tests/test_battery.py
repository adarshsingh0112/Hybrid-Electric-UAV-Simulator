import pytest
import sys
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from battery.battery import Battery


def test_battery_energy_depletion_physical():
    """Verify battery energy depletion uses physical Energy = (Power * time) / discharge_efficiency."""
    battery = Battery()
    initial_energy = battery.energy_remaining

    power_kw = 50.0
    duration_min = 60  # 1 hr
    battery.consume_energy(power_kw, duration_min)

    expected_energy_used = (power_kw * 1.0) / battery.discharge_efficiency  # 50 / 0.96 = 52.0833 kWh
    expected_remaining = initial_energy - expected_energy_used

    assert pytest.approx(battery.energy_remaining, abs=1e-3) == expected_remaining


def test_battery_soc_bounds():
    """Verify battery State of Charge (SoC) stays strictly between 0 and 100 percent."""
    battery = Battery()

    # Deplete heavily
    battery.consume_energy(250.0, 1000)
    assert 0.0 <= battery.soc <= 100.0, f"SoC dropped outside [0, 100]: {battery.soc}"

    # Recharge heavily
    battery.recharge(250.0, 1000)
    assert 0.0 <= battery.soc <= 100.0, f"SoC exceeded 100%: {battery.soc}"


def test_battery_recharge_efficiency():
    """Verify battery recharge energy added accounts for charging efficiency."""
    battery = Battery()
    battery.soc = 50.0
    battery.energy_remaining = 110.0  # half capacity

    power_kw = 50.0
    duration_min = 60  # 1 hr
    battery.recharge(power_kw, duration_min)

    expected_energy_added = 50.0 * 1.0 * battery.charge_efficiency  # 50 * 0.95 = 47.5 kWh
    assert pytest.approx(battery.energy_remaining, abs=1e-3) == 110.0 + expected_energy_added
