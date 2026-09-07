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


def test_mass_increases_required_lift():
    """Verify higher aircraft mass increases required lift."""
    aircraft = Aircraft()

    aero_light = aircraft.calculate_aerodynamics(altitude_m=5000, speed_kmph=200, mass_kg=800)
    aero_heavy = aircraft.calculate_aerodynamics(altitude_m=5000, speed_kmph=200, mass_kg=1100)

    assert aero_heavy["cl"] > aero_light["cl"], "Higher mass must result in higher lift coefficient CL."
    assert aero_heavy["drag_n"] > aero_light["drag_n"], "Higher mass must result in higher total drag."


def test_speed_changes_drag_and_induced_drag_decrease():
    """Verify speed changes drag and induced drag CDi decreases with higher speed."""
    aircraft = Aircraft()

    # At low speed (120 km/h), CL is high so CDi is high
    aero_slow = aircraft.calculate_aerodynamics(altitude_m=5000, speed_kmph=120, mass_kg=1000)

    # At higher speed (250 km/h), CL is lower so CDi decreases
    aero_fast = aircraft.calculate_aerodynamics(altitude_m=5000, speed_kmph=250, mass_kg=1000)

    assert aero_fast["cdi"] < aero_slow["cdi"], (
        f"Induced drag CDi should decrease at higher speeds. "
        f"Slow CDi: {aero_slow['cdi']}, Fast CDi: {aero_fast['cdi']}"
    )


def test_drag_and_power_non_negative():
    """Verify drag and required propulsive power are non-negative."""
    aircraft = Aircraft()

    for alt in [0, 1000, 5000, 10000]:
        for speed in [85, 150, 250, 320]:
            aero = aircraft.calculate_aerodynamics(altitude_m=alt, speed_kmph=speed, mass_kg=1000)

            assert aero["drag_n"] >= 0.0, f"Drag was negative: {aero['drag_n']}"
            assert aero["propulsive_power_kw"] >= 0.0, f"Propulsive power was negative: {aero['propulsive_power_kw']}"
            assert aero["shaft_power_kw"] >= 0.0, f"Shaft power was negative: {aero['shaft_power_kw']}"
            assert aero["motor_elec_power_kw"] >= 0.0, f"Motor electrical power was negative: {aero['motor_elec_power_kw']}"


def test_climb_requires_more_power_than_level():
    """Verify climbing flight requires more power than equivalent level flight."""
    aircraft = Aircraft()

    aero_level = aircraft.calculate_aerodynamics(altitude_m=300, speed_kmph=180, mass_kg=1000, climb_rate_ms=0.0)
    aero_climb = aircraft.calculate_aerodynamics(altitude_m=300, speed_kmph=180, mass_kg=1000, climb_rate_ms=5.0)

    assert aero_climb["motor_elec_power_kw"] > aero_level["motor_elec_power_kw"], (
        f"Climb power ({aero_climb['motor_elec_power_kw']} kW) must be greater than level flight ({aero_level['motor_elec_power_kw']} kW)"
    )


def test_propulsion_power_chain_domain_separation():
    """Verify strict separation and efficiency relations: P_prop < P_shaft < P_elec."""
    aircraft = Aircraft()
    aero = aircraft.calculate_aerodynamics(altitude_m=5000, speed_kmph=250, mass_kg=1000)

    p_prop = aero["propulsive_power_kw"]
    p_shaft = aero["shaft_power_kw"]
    p_elec = aero["motor_elec_power_kw"]

    assert pytest.approx(p_shaft * aircraft.eta_propeller, abs=1e-2) == p_prop
    assert pytest.approx(p_elec * aircraft.eta_motor, abs=1e-2) == p_shaft
    assert p_prop < p_shaft < p_elec, f"Power domain hierarchy broken: P_prop={p_prop}, P_shaft={p_shaft}, P_elec={p_elec}"


def test_end_to_end_atmosphere_aerodynamics_power_manager_pipeline():
    """Integration test verifying: Altitude -> Atmosphere -> Density -> Aerodynamics -> Required Power -> Power Manager."""
    aircraft = Aircraft()
    battery = Battery()
    engine = Engine()
    motor = Motor()

    engine.start()
    motor.start()

    power_manager = PowerManager(battery, engine, motor)

    altitude_m = 5000
    speed_kmph = 250
    mass_kg = aircraft.get_current_mass(engine.fuel_remaining)

    # 1. Aerodynamics & Atmosphere
    aero = aircraft.calculate_aerodynamics(altitude_m, speed_kmph, mass_kg)
    p_req_elec = aero["motor_elec_power_kw"]

    assert aero["density_kg_m3"] < 1.225
    assert p_req_elec > 0.0

    # 2. Power Manager Execution
    engine_elec, battery_elec = power_manager.execute("Cruise", p_req_elec, duration=10)

    # 3. Verify Power Conservation
    assert pytest.approx(engine_elec + battery_elec, abs=1e-4) == p_req_elec
