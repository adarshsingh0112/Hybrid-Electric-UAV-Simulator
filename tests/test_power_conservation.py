import pytest
import sys
from pathlib import Path

# Add src to sys.path for test discovery
SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from battery.battery import Battery
from engine.engine import Engine
from motor.motor import Motor
from power_manager.power_manager import PowerManager


def test_power_conservation_exact():
    """Verify P_engine_elec + P_battery_elec == P_required within 1e-4 kW."""
    battery = Battery()
    engine = Engine()
    motor = Motor()

    engine.start()
    motor.start()

    power_manager = PowerManager(battery, engine, motor)

    test_powers = [50.0, 100.0, 162.5, 225.0, 250.0]
    duration = 10  # min

    for p_req in test_powers:
        engine_elec, battery_elec = power_manager.execute("Cruise", p_req, duration)
        total_supplied = engine_elec + battery_elec

        assert pytest.approx(total_supplied, abs=1e-4) == p_req, (
            f"Power conservation violated! Required: {p_req} kW, "
            f"Engine Elec: {engine_elec} kW, Battery Elec: {battery_elec} kW"
        )


def test_generator_efficiency_application():
    """Verify engine electrical output power is engine_mech * generator_efficiency."""
    engine = Engine()
    engine.start()

    engine.set_power(200.0)  # 200 kW mechanical
    gen_eff = engine.generator_efficiency

    expected_elec = 200.0 * gen_eff
    actual_elec = engine.get_electrical_power()

    assert pytest.approx(actual_elec, abs=1e-4) == expected_elec, (
        f"Generator efficiency not applied correctly! Mech: 200 kW, Gen Eff: {gen_eff}, "
        f"Expected Elec: {expected_elec} kW, Actual: {actual_elec} kW"
    )


def test_power_deficit_reporting(capsys):
    """Verify explicit power deficit report when required power exceeds engine + battery max."""
    battery = Battery()
    engine = Engine()
    motor = Motor()

    engine.start()
    motor.start()

    power_manager = PowerManager(battery, engine, motor)

    # Exceed maximum combined power capability (e.g. 600 kW required)
    huge_power_req = 600.0
    duration = 5

    engine_elec, battery_elec = power_manager.execute("Takeoff", huge_power_req, duration)
    captured = capsys.readouterr().out

    assert "POWER DEFICIT" in captured, "Power deficit warning was not logged when limits were exceeded!"
    assert engine_elec + battery_elec < huge_power_req, "Total power should be clamped to max capabilities."
