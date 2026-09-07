import pytest
import sys
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from aircraft.aircraft import Aircraft
from battery.battery import Battery
from engine.engine import Engine
from motor.motor import Motor
from power_manager.power_manager import PowerManager
from controller.hybrid_uav import HybridUAV


def test_engine_cannot_exceed_max_mechanical_power():
    """1. Verify engine mechanical power cannot exceed rated max limit (300 kW)."""
    engine = Engine()
    engine.start()
    engine.set_power(500.0)

    assert engine.current_power == engine.max_power == 300.0


def test_engine_fuel_consumption_non_negative():
    """2. Verify engine fuel consumption is non-negative."""
    engine = Engine()
    engine.start()
    engine.set_power(100.0)
    fuel_used = engine.consume_fuel(10)

    assert fuel_used >= 0.0


def test_bsfc_unit_formula_numerical_verification():
    """Validate BSFC formula: fuel_used_kg = (power_kw * bsfc_g_per_kwh * dt_sec) / 3,600,000."""
    engine = Engine()
    engine.start()

    pwr_kw = 200.0
    bsfc_g = engine.bsfc_g_per_kwh  # 180.0 g/kWh
    duration_min = 30.0
    dt_sec = duration_min * 60.0    # 1800 sec

    # Expected formula value: (200 * 180 * 1800) / 3,600,000 = 64,800,000 / 3,600,000 = 18.0 kg
    expected_kg = (pwr_kw * bsfc_g * dt_sec) / 3_600_000.0

    engine.set_power(pwr_kw)
    actual_used_kg = engine.consume_fuel(duration_min)

    assert pytest.approx(actual_used_kg, abs=1e-3) == expected_kg == 18.0


def test_generator_electrical_output_conversion_and_rating_limit():
    """3 & 4. Verify generator electrical output equals mechanical * efficiency, clamped to max electrical rating."""
    engine = Engine()
    engine.start()

    # Case A: Below rating limit
    engine.set_power(100.0)
    expected_elec = 100.0 * engine.generator_efficiency  # 93.0 kW
    assert pytest.approx(engine.get_electrical_power(), abs=1e-4) == expected_elec

    # Case B: Rating limit saturation
    engine.generator_max_elec_kw = 200.0  # Set custom lower rating limit
    engine.set_power(300.0)
    assert engine.get_electrical_power() == 200.0
    assert engine.is_generator_saturated()


def test_battery_cannot_exceed_max_discharge_power():
    """5. Verify battery discharge power is clamped to maximum discharge limit (250 kW)."""
    battery = Battery()
    delivered = battery.consume_energy(400.0, 10)  # Request 400 kW

    expected_power = battery.max_discharge_power   # 250 kW
    assert pytest.approx(delivered, abs=1e-2) == expected_power


def test_battery_reserve_soc_energy_limit():
    """6 & 7. Verify battery cannot exceed usable energy (min_soc reserve protected) and SOC stays in [0, 100]."""
    battery = Battery()

    # Discharge heavily
    battery.consume_energy(250.0, 1000)
    assert battery.soc >= battery.min_soc == 20.0
    assert battery.get_available_discharge_power() == 0.0


def test_battery_discharge_and_charge_energy_conservation():
    """Validate energy conservation during battery discharge and battery charging."""
    battery = Battery()
    initial_energy = battery.energy_remaining  # 220 kWh

    # Discharge 100 kW for 30 min (0.5 hr)
    # Energy drawn from storage = (100 * 0.5) / 0.96 = 52.0833 kWh
    battery.consume_energy(100.0, 30.0)
    energy_after_dis = battery.energy_remaining
    assert pytest.approx(energy_after_dis, abs=1e-3) == initial_energy - (50.0 / 0.96)

    # Charge 50 kW for 30 min (0.5 hr)
    # Energy added to storage = (50 * 0.5) * 0.95 = 23.75 kWh
    battery.recharge(50.0, 30.0)
    energy_after_chg = battery.energy_remaining
    assert pytest.approx(energy_after_chg, abs=1e-3) == energy_after_dis + 23.75


def test_motor_electrical_to_mechanical_shaft_conversion_and_limit():
    """8 & 9. Verify P_shaft = P_elec * eta_motor and motor max power clamping."""
    motor = Motor()
    motor.start()

    # Set power within limit
    motor.set_power(100.0)
    assert motor.current_elec_power == 100.0
    assert pytest.approx(motor.get_shaft_power(), abs=1e-4) == 100.0 * motor.efficiency == 95.0

    # Set power above max limit (300 kW)
    motor.set_power(300.0)
    assert motor.current_elec_power == motor.max_elec_power == 250.0
    assert motor.is_motor_saturated()


def test_propulsion_power_deficit_detection():
    """10. Verify propulsion power deficit is detected when required power exceeds limits."""
    battery = Battery()
    engine = Engine()
    motor = Motor()

    engine.start()
    motor.start()

    power_manager = PowerManager(battery, engine, motor)

    # Required power 600 kW exceeds total available (279 kW gen + 250 kW batt = 529 kW)
    engine_elec, battery_elec = power_manager.execute("Takeoff", 600.0, 5)

    assert engine_elec + battery_elec < 600.0


def test_power_manager_dc_bus_conservation():
    """11. Verify Power Manager conserves electrical bus power: P_gen + P_bat + P_def = P_bus."""
    battery = Battery()
    engine = Engine()
    motor = Motor()

    engine.start()
    motor.start()

    power_manager = PowerManager(battery, engine, motor)

    for req_pwr in [40.0, 100.0, 180.0, 250.0]:
        engine_elec, battery_elec = power_manager.execute("Cruise", req_pwr, 10)
        supplied_bus = engine_elec + battery_elec
        assert pytest.approx(supplied_bus, abs=1e-4) == req_pwr


def test_no_negative_fuel():
    """12. Verify fuel remaining never drops below zero."""
    engine = Engine()
    engine.start()
    engine.set_power(300.0)
    engine.consume_fuel(10000)

    assert engine.fuel_remaining == 0.0


def test_fuel_mass_affects_aircraft_mass():
    """13. Verify fuel consumption directly updates dynamic aircraft mass."""
    aircraft = Aircraft()
    engine = Engine()

    initial_fuel = engine.fuel_remaining  # 250 kg
    initial_mass = aircraft.get_current_mass(initial_fuel)

    engine.start()
    engine.set_power(200.0)
    fuel_used = engine.consume_fuel(60)  # 1 hour run

    new_fuel = engine.fuel_remaining
    new_mass = aircraft.get_current_mass(new_fuel)

    assert new_fuel == initial_fuel - fuel_used
    assert pytest.approx(new_mass, abs=1e-4) == initial_mass - fuel_used


def test_full_mission_simulation_run():
    """14. Verify full mission simulation executes successfully end-to-end."""
    uav = HybridUAV()
    uav.initialize()
    uav.fly()

    assert uav.engine.fuel_remaining > 0.0
    assert uav.battery.soc >= uav.battery.min_soc
