import time
from typing import Optional

from aircraft.aircraft import Aircraft
from telemetry.telemetry import Telemetry
try:
    from src.dashboard.dashboard import Dashboard
except ImportError:
    from uav_dashboard.dashboard import Dashboard
from mission.state import SimulationState


class MissionSimulator:

    def __init__(
        self,
        mission,
        power_manager,
        aircraft: Optional[Aircraft] = None,
        dt_s: float = 1.0
    ):
        self.mission = mission
        self.power_manager = power_manager
        self.aircraft = aircraft if aircraft is not None else Aircraft()
        self.dt_s = float(dt_s)

        # Kinematic acceleration bounds
        self.max_accel_mps2 = 1.0   # Maximum acceleration (+1.0 m/s^2)
        self.max_decel_mps2 = -0.5  # Maximum deceleration (-0.5 m/s^2)

        # Simulation state timers & kinematics
        self.mission_time_s = 0.0
        self.mission_time = 0.0  # Legacy compatibility (minutes)
        self.current_altitude_m = 0.0
        self.current_airspeed_mps = 0.0

        # Telemetry logger & Dashboard
        self.telemetry = Telemetry()
        self.dashboard = Dashboard()

    # ---------------------------------------------------------

    def run(self):
        print("\n" + "=" * 65)
        print(f"MISSION START (FIXED TIME-STEP STEP SIMULATION, dt = {self.dt_s:.2f} s)")
        print("=" * 65)

        # Ensure engine is started
        if not self.power_manager.engine.running:
            self.power_manager.engine.start()

        for phase in self.mission.phases:
            phase_name = phase["name"]
            duration_min = float(phase["duration_min"])
            phase_duration_s = duration_min * 60.0

            target_speed_kmph = float(phase["target_speed_kmph"])
            target_speed_mps = target_speed_kmph / 3.6
            target_altitude_m = float(phase["target_altitude_m"])

            # Determine climb rate Vz
            climb_rate_ms = 0.0
            if "climb_rate_ms" in phase:
                climb_rate_ms = float(phase["climb_rate_ms"])
            elif phase_name == "Takeoff":
                climb_rate_ms = 1.0
            elif phase_name == "Climb":
                climb_rate_ms = 5.22
            elif phase_name == "Descent":
                climb_rate_ms = -3.0

            print(
                f"\n[PHASE START] {phase_name:<10} | Target Speed: {target_speed_kmph:5.1f} km/h | "
                f"Target Alt: {target_altitude_m:6.1f} m | Duration: {duration_min:4.1f} min"
            )

            phase_elapsed_s = 0.0

            # Exact-duration simulation loop
            while phase_elapsed_s < (phase_duration_s - 1e-6):
                step_dt = min(self.dt_s, phase_duration_s - phase_elapsed_s)

                # 1. Altitude propagation
                if climb_rate_ms > 0:
                    self.current_altitude_m = min(
                        target_altitude_m,
                        self.current_altitude_m + climb_rate_ms * step_dt
                    )
                elif climb_rate_ms < 0:
                    self.current_altitude_m = max(
                        target_altitude_m,
                        self.current_altitude_m + climb_rate_ms * step_dt
                    )
                else:
                    if abs(self.current_altitude_m - target_altitude_m) > 1e-3:
                        direction = 1.0 if target_altitude_m > self.current_altitude_m else -1.0
                        self.current_altitude_m += direction * min(
                            5.0 * step_dt,
                            abs(target_altitude_m - self.current_altitude_m)
                        )
                    else:
                        self.current_altitude_m = target_altitude_m

                # 2. Airspeed propagation with kinematic bounds
                speed_diff = target_speed_mps - self.current_airspeed_mps
                if speed_diff > 0:
                    max_inc = self.max_accel_mps2 * step_dt
                    self.current_airspeed_mps += min(speed_diff, max_inc)
                elif speed_diff < 0:
                    max_dec = abs(self.max_decel_mps2) * step_dt
                    self.current_airspeed_mps -= min(abs(speed_diff), max_dec)

                # 3. Aircraft total mass (dry + payload + battery + remaining fuel)
                current_mass = self.aircraft.get_current_mass(
                    self.power_manager.engine.fuel_remaining
                )

                # 4. Aerodynamics and electrical demand calculation
                aero = self.aircraft.calculate_aerodynamics(
                    altitude_m=self.current_altitude_m,
                    speed_kmph=self.current_airspeed_mps * 3.6,
                    mass_kg=current_mass,
                    climb_rate_ms=climb_rate_ms
                )
                required_power = aero["motor_elec_power_kw"]

                # 5. Hybrid power dispatch
                engine_elec, battery_elec = self.power_manager.execute(
                    mission_phase=phase_name,
                    required_power=required_power,
                    duration=step_dt,
                    is_seconds=True,
                    verbose=False
                )

                power_deficit = getattr(self.power_manager, "last_power_deficit", 0.0)
                is_power_limited = getattr(self.power_manager, "last_is_power_limited", False)

                # 6. Record SimulationState
                next_time_s = round(self.mission_time_s + step_dt, 6)
                state = SimulationState(
                    time_s=next_time_s,
                    mission_phase=phase_name,
                    altitude_m=self.current_altitude_m,
                    target_altitude_m=target_altitude_m,
                    airspeed_mps=self.current_airspeed_mps,
                    target_airspeed_mps=target_speed_mps,
                    air_density_kg_m3=aero["density_kg_m3"],
                    aircraft_mass_kg=current_mass,
                    remaining_fuel_kg=self.power_manager.engine.fuel_remaining,
                    battery_soc_percent=self.power_manager.battery.soc,
                    required_propulsive_power_kw=aero["propulsive_power_kw"],
                    required_motor_elec_power_kw=required_power,
                    engine_mechanical_power_kw=self.power_manager.engine.current_power,
                    generator_electrical_power_kw=engine_elec,
                    battery_power_kw=battery_elec,
                    motor_elec_power_kw=self.power_manager.motor.current_elec_power,
                    motor_shaft_power_kw=self.power_manager.motor.get_shaft_power(),
                    power_deficit_kw=power_deficit,
                    is_power_limited=is_power_limited,
                    motor_rpm=self.power_manager.motor.current_rpm
                )

                self.telemetry.log_state(state)

                # Advance timer
                phase_elapsed_s = round(phase_elapsed_s + step_dt, 6)
                self.mission_time_s = round(self.mission_time_s + step_dt, 6)
                self.mission_time = self.mission_time_s / 60.0

            print(
                f"[PHASE END]   {phase_name:<10} | Sim Time: {self.mission_time:5.1f} min | "
                f"Altitude: {self.current_altitude_m:6.1f} m | SoC: {self.power_manager.battery.soc:5.1f}% | "
                f"Fuel Left: {self.power_manager.engine.fuel_remaining:6.2f} kg"
            )

        # Save mission telemetry CSV
        self.telemetry.save_csv("mission_log.csv")

        summary = self.telemetry.get_summary()

        print("\n" + "=" * 65)
        print("MISSION COMPLETED SUCCESSFULLY")
        print("-" * 65)
        print(f"Total Flight Time     : {summary.get('total_flight_time_min', 0.0):.2f} min ({summary.get('total_flight_time_s', 0.0):.1f} s)")
        print(f"Total Telemetry Rows  : {summary.get('record_count', 0)}")
        print(f"Fuel Consumed         : {summary.get('fuel_consumed_kg', 0.0):.2f} kg (Final Remaining: {summary.get('final_fuel_kg', 0.0):.2f} kg)")
        print(f"Final Battery SoC     : {summary.get('final_soc_percent', 0.0):.1f}%")
        print(f"Power Limited Seconds : {summary.get('power_limited_seconds', 0.0):.1f} s")
        print(f"Total Deficit Energy  : {summary.get('total_deficit_energy_kwh', 0.0):.4f} kWh")
        print(f"Max Power Deficit     : {summary.get('max_power_deficit_kw', 0.0):.2f} kW")
        print("=" * 65)

        return summary