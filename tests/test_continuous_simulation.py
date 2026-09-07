import pytest
import sys
from pathlib import Path

# Add src to pythonpath
src_path = Path(__file__).resolve().parent.parent / "src"
if str(src_path) not in sys.path:
    sys.path.insert(0, str(src_path))

from aircraft.aircraft import Aircraft
from battery.battery import Battery
from engine.engine import Engine
from motor.motor import Motor
from power_manager.power_manager import PowerManager
from mission.mission import Mission
from mission.simulator import MissionSimulator
from mission.state import SimulationState
from telemetry.telemetry import Telemetry


def test_simulation_state_initialization():
    state = SimulationState(
        time_s=10.0,
        mission_phase="Climb",
        altitude_m=500.0,
        target_altitude_m=1000.0,
        airspeed_mps=50.0,
        target_airspeed_mps=60.0,
        air_density_kg_m3=1.167,
        aircraft_mass_kg=1620.0,
        remaining_fuel_kg=250.0,
        battery_soc_percent=95.0,
        required_propulsive_power_kw=80.0,
        required_motor_elec_power_kw=100.0,
        engine_mechanical_power_kw=100.0,
        generator_electrical_power_kw=93.0,
        battery_power_kw=7.0,
        motor_elec_power_kw=100.0,
        motor_shaft_power_kw=95.0,
        power_deficit_kw=0.0,
        is_power_limited=False
    )
    assert state.time_s == 10.0
    assert state.aircraft_mass_kg == 1620.0
    assert state.to_dict()["Phase"] == "Climb"


def test_aircraft_mass_includes_battery_mass():
    aircraft = Aircraft()
    # empty=650, payload=200, battery=520, fuel=250 -> MTOW=1620
    mass = aircraft.get_current_mass(fuel_remaining_kg=250.0)
    assert mass == 650.0 + 200.0 + 520.0 + 250.0
    assert mass == 1620.0

    mass_depleted = aircraft.get_current_mass(fuel_remaining_kg=0.0)
    assert mass_depleted == 650.0 + 200.0 + 520.0
    assert mass_depleted == 1370.0


def test_fixed_timestep_propagation():
    mission = Mission()
    power_manager = PowerManager(Battery(), Engine(), Motor())
    simulator = MissionSimulator(mission, power_manager, dt_s=1.0)
    summary = simulator.run()

    total_mission_min = sum(p["duration_min"] for p in mission.phases)
    expected_duration_s = total_mission_min * 60.0

    assert summary["total_flight_time_s"] == expected_duration_s
    assert summary["record_count"] == int(expected_duration_s)


def test_exact_duration_loop_execution():
    mission = Mission()
    power_manager = PowerManager(Battery(), Engine(), Motor())
    # Test non-integer step fraction dt_s = 7.0 s
    simulator = MissionSimulator(mission, power_manager, dt_s=7.0)
    summary = simulator.run()

    total_mission_min = sum(p["duration_min"] for p in mission.phases)
    expected_duration_s = total_mission_min * 60.0

    assert pytest.approx(summary["total_flight_time_s"], abs=1e-3) == expected_duration_s


def test_altitude_climb_propagation():
    mission = Mission()
    power_manager = PowerManager(Battery(), Engine(), Motor())
    simulator = MissionSimulator(mission, power_manager, dt_s=1.0)
    simulator.run()

    altitudes = [s.altitude_m for s in simulator.telemetry.states]
    # Check that altitude starts at 0 and increases smoothly
    assert altitudes[0] == 1.0  # after 1 second of takeoff climb rate 1.0 m/s
    assert max(altitudes) <= 10000.0
    for i in range(1, 100):
        # Altitude increment per second during climb must not jump drastically
        assert (altitudes[i] - altitudes[i - 1]) <= 5.23


def test_airspeed_acceleration_bounds():
    mission = Mission()
    power_manager = PowerManager(Battery(), Engine(), Motor())
    simulator = MissionSimulator(mission, power_manager, dt_s=1.0)
    simulator.run()

    speeds = [s.airspeed_mps for s in simulator.telemetry.states]
    for i in range(1, len(speeds)):
        delta_v = speeds[i] - speeds[i - 1]
        assert delta_v <= 1.001   # Max accel bound (+1.0 m/s^2)
        assert delta_v >= -0.501  # Max decel bound (-0.5 m/s^2)


def test_continuous_fuel_consumption():
    engine = Engine()
    engine.start()
    engine.set_power(200.0)  # 200 kW

    fuel_start = engine.fuel_remaining
    engine.consume_fuel(1.0, is_seconds=True)
    fuel_after_1s = engine.fuel_remaining

    expected_used = (200.0 * engine.bsfc_g_per_kwh * 1.0) / 3_600_000.0
    actual_used = fuel_start - fuel_after_1s
    assert pytest.approx(actual_used, abs=1e-6) == expected_used


def test_continuous_battery_soc():
    battery = Battery()
    soc_start = battery.soc
    battery.consume_energy(100.0, 1.0, is_seconds=True)
    assert battery.soc < soc_start


def test_reserve_soc_protection():
    battery = Battery()
    battery.soc = 20.0  # Reserve limit reached
    power_delivered = battery.consume_energy(50.0, 1.0, is_seconds=True)
    assert power_delivered == 0.0
    assert battery.get_available_discharge_power() == 0.0


def test_power_deficit_handling():
    battery = Battery()
    battery.soc = 20.0  # Battery empty/reserved
    engine = Engine()
    engine.start()
    engine.fuel_remaining = 0.0  # Fuel empty -> engine electrical output is 0.0 kW
    motor = Motor()
    pm = PowerManager(battery, engine, motor)

    # Demand 200 kW on DC bus -> deficit of 200 kW (engine empty & battery reserved)
    pm.execute("Cruise", 200.0, 1.0, is_seconds=True, verbose=False)

    assert pm.last_is_power_limited is True
    assert pm.last_power_deficit == 200.0


def test_telemetry_high_frequency_logging():
    telemetry = Telemetry()
    state = SimulationState(
        time_s=1.0,
        mission_phase="Takeoff",
        altitude_m=1.0,
        target_altitude_m=10.0,
        airspeed_mps=1.0,
        target_airspeed_mps=10.0,
        air_density_kg_m3=1.225,
        aircraft_mass_kg=1620.0,
        remaining_fuel_kg=250.0,
        battery_soc_percent=100.0,
        required_propulsive_power_kw=10.0,
        required_motor_elec_power_kw=12.0,
        engine_mechanical_power_kw=40.0,
        generator_electrical_power_kw=37.2,
        battery_power_kw=0.0,
        motor_elec_power_kw=12.0,
        motor_shaft_power_kw=11.4,
        power_deficit_kw=0.0,
        is_power_limited=False
    )
    telemetry.log_state(state)
    assert len(telemetry.records) == 1
    assert "Aircraft Mass (kg)" in telemetry.records[0]
    assert telemetry.records[0]["Aircraft Mass (kg)"] == 1620.0


def test_different_timestep_resolutions():
    mission1 = Mission()
    pm1 = PowerManager(Battery(), Engine(), Motor())
    sim1 = MissionSimulator(mission1, pm1, dt_s=1.0)
    sum1 = sim1.run()

    mission2 = Mission()
    pm2 = PowerManager(Battery(), Engine(), Motor())
    sim2 = MissionSimulator(mission2, pm2, dt_s=0.5)
    sum2 = sim2.run()

    assert pytest.approx(sum1["fuel_consumed_kg"], rel=1e-2) == sum2["fuel_consumed_kg"]
    assert pytest.approx(sum1["final_soc_percent"], rel=1e-2) == sum2["final_soc_percent"]


def test_phase_transition_continuity():
    mission = Mission()
    power_manager = PowerManager(Battery(), Engine(), Motor())
    simulator = MissionSimulator(mission, power_manager, dt_s=1.0)
    simulator.run()

    for s in simulator.telemetry.states:
        assert not sys.is_nan(s.altitude_m) if hasattr(sys, "is_nan") else (s.altitude_m == s.altitude_m)
        assert s.aircraft_mass_kg >= 1370.0  # Mass never drops below dry+payload+battery


def test_telemetry_summary_statistics():
    mission = Mission()
    power_manager = PowerManager(Battery(), Engine(), Motor())
    simulator = MissionSimulator(mission, power_manager, dt_s=1.0)
    summary = simulator.run()

    assert "total_flight_time_s" in summary
    assert "fuel_consumed_kg" in summary
    assert "power_limited_seconds" in summary
    assert summary["fuel_consumed_kg"] > 0.0
