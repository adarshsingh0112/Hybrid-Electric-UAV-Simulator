import pytest
import sys
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from engine.engine import Engine


def test_engine_minimum_power_clamping():
    """Verify engine power cannot drop below idle_power (40 kW) when running."""
    engine = Engine()
    engine.start()

    # Request power below idle (e.g. 10 kW)
    engine.set_power(10.0)

    assert engine.current_power == engine.idle_power, (
        f"Engine power should be clamped to idle ({engine.idle_power} kW), but got {engine.current_power} kW"
    )


def test_engine_maximum_power_clamping():
    """Verify engine power cannot exceed max_power (300 kW)."""
    engine = Engine()
    engine.start()

    # Request power above max (e.g. 500 kW)
    engine.set_power(500.0)

    assert engine.current_power == engine.max_power, (
        f"Engine power should be clamped to max ({engine.max_power} kW), but got {engine.current_power} kW"
    )


def test_engine_no_negative_fuel():
    """Verify fuel remaining never drops below 0 kg."""
    engine = Engine()
    engine.start()
    engine.set_power(300.0)

    # Consume fuel for an excessively long duration (e.g. 10,000 minutes)
    engine.consume_fuel(10000)

    assert engine.fuel_remaining == 0.0, (
        f"Fuel remaining should be 0.0 kg when depleted, but got {engine.fuel_remaining} kg"
    )
    assert not engine.running, "Engine should stop automatically when fuel is depleted."


def test_engine_fuel_consumption_rate():
    """Verify fuel consumption matches P * SFC * time."""
    engine = Engine()
    engine.start()
    engine.set_power(100.0)  # 100 kW

    duration_min = 60  # 1 hour
    fuel_used = engine.consume_fuel(duration_min)

    expected_fuel_used = 100.0 * engine.sfc * 1.0  # 18.0 kg
    assert pytest.approx(fuel_used, abs=1e-2) == expected_fuel_used
    assert pytest.approx(engine.fuel_remaining, abs=1e-2) == (250.0 - expected_fuel_used)
