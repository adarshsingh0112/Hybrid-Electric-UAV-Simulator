"""
Monte Carlo Scenario Generator Module (Step 9)
Generates physical uncertainty scenarios for UAV Predictive Fast-Forward Simulation.

Uncertainty Factors Modelled:
1. Wind Vector: Speed and meteorological direction (affects ground speed via vector triangle).
2. Ambient Temperature: Deviation from standard ISA (affects air density, cooling, and battery resistance).
3. Engine Degradation: Turbo derating, cylinder wear, and specific fuel consumption penalties.
4. Battery Degradation & Estimation Uncertainty: Internal resistance growth and initial SoC state error.
5. Propulsion Power Demand: Aerodynamic drag fluctuation and surface roughness.
6. Payload Mass: Variations in sensor, weapon, or ballast mass.

Deterministic & Stochastic Scenarios Supported:
- Nominal Scenario (zero perturbation)
- Best-Case Scenario (optimal bounded atmospheric and mechanical state)
- Worst-Case Scenario (severe headwind, elevated temperature, maximal degradation)
- Monte Carlo Stochastic Ensemble (reproducible with fixed seed)
"""

from dataclasses import dataclass, field, asdict
from enum import Enum
import json
import math
from pathlib import Path
import random
from typing import Dict, List, Optional, Tuple, Any, Union


class ScenarioType(str, Enum):
    NOMINAL = "NOMINAL"
    BEST_CASE = "BEST_CASE"
    WORST_CASE = "WORST_CASE"
    MONTE_CARLO = "MONTE_CARLO"


@dataclass
class ScenarioParameters:
    """
    Physical parameter perturbations defining a single future scenario.
    All parameters are physically meaningful and documented.
    """
    scenario_id: str
    scenario_type: ScenarioType
    wind_speed_mps: float              # Wind speed magnitude [m/s]
    wind_direction_deg: float          # Direction FROM which wind blows [0 - 360 deg]
    ambient_temp_delta_c: float        # OAT deviation from standard ISA [deg C]
    engine_power_multiplier: float     # Engine maximum power capacity multiplier [0.80 - 1.05]
    engine_fuel_flow_multiplier: float # Engine fuel consumption rate multiplier [0.95 - 1.20]
    battery_resistance_multiplier: float # Pack internal resistance multiplier [0.85 - 1.50]
    battery_initial_soc_offset_pct: float # Kalman Filter state estimation error offset [% SoC]
    propulsion_power_multiplier: float # Aerodynamic drag & airframe roughness multiplier [0.90 - 1.15]
    payload_mass_delta_kg: float       # Payload mass deviation from nominal 1100 kg [-50 to +50 kg]
    description: str = ""

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["scenario_type"] = self.scenario_type.value
        return d


@dataclass
class FastForwardConfig:
    """
    Configuration specs and uncertainty bounds for Fast-Forward Twin simulation.
    """
    policy_name: str = "DRDO-MALE-UAV-Fast-Forward-Monte-Carlo-v1"
    num_scenarios: int = 20
    simulation_timestep_s: float = 2.0
    max_simulation_duration_s: float = 7200.0
    random_seed: int = 42
    max_computation_budget_ms: float = 250.0
    periodic_update_interval_s: float = 5.0
    
    # Wind uncertainty
    wind_mean_speed_mps: float = 0.0
    wind_std_speed_mps: float = 4.0
    wind_max_speed_mps: float = 15.0

    # Ambient temperature uncertainty
    temp_mean_delta_c: float = 0.0
    temp_std_delta_c: float = 2.5
    temp_min_delta_c: float = -8.0
    temp_max_delta_c: float = 8.0

    # Engine degradation
    engine_min_power_mult: float = 0.85
    engine_max_power_mult: float = 1.00
    engine_min_fuel_mult: float = 1.00
    engine_max_fuel_mult: float = 1.10

    # Battery uncertainty
    battery_min_r_mult: float = 0.95
    battery_max_r_mult: float = 1.30
    battery_max_soc_offset_pct: float = 2.0

    # Power demand & mass
    power_demand_min_mult: float = 0.95
    power_demand_max_mult: float = 1.08
    base_uav_mass_kg: float = 1100.0
    payload_mass_min_delta_kg: float = -25.0
    payload_mass_max_delta_kg: float = 25.0

    # Safety thresholds
    min_fuel_reserve_liters: float = 18.0
    min_battery_reserve_pct: float = 20.0
    critical_battery_temp_c: float = 65.0
    critical_engine_cht_c: float = 220.0
    max_permitted_power_deficit_w: float = 8000.0

    # Statistical reporting
    percentiles: List[float] = field(default_factory=lambda: [5.0, 50.0, 95.0])
    confidence_level: float = 0.95

    @classmethod
    def from_json(cls, json_path: Union[str, Path]) -> "FastForwardConfig":
        path = Path(json_path)
        if not path.exists():
            raise FileNotFoundError(f"Configuration file not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        flat: Dict[str, Any] = {}
        if "policy_name" in data:
            flat["policy_name"] = str(data["policy_name"])
        if "num_scenarios" in data:
            flat["num_scenarios"] = int(data["num_scenarios"])
        if "simulation_timestep_s" in data:
            flat["simulation_timestep_s"] = float(data["simulation_timestep_s"])
        if "max_simulation_duration_s" in data:
            flat["max_simulation_duration_s"] = float(data["max_simulation_duration_s"])
        if "random_seed" in data:
            flat["random_seed"] = int(data["random_seed"])
        if "max_computation_budget_ms" in data:
            flat["max_computation_budget_ms"] = float(data["max_computation_budget_ms"])
        if "periodic_update_interval_s" in data:
            flat["periodic_update_interval_s"] = float(data["periodic_update_interval_s"])

        wind = data.get("wind_uncertainty", {})
        if "mean_speed_mps" in wind:
            flat["wind_mean_speed_mps"] = float(wind["mean_speed_mps"])
        if "std_speed_mps" in wind:
            flat["wind_std_speed_mps"] = float(wind["std_speed_mps"])
        if "max_speed_mps" in wind:
            flat["wind_max_speed_mps"] = float(wind["max_speed_mps"])

        temp = data.get("ambient_temp_uncertainty", {})
        if "mean_delta_c" in temp:
            flat["temp_mean_delta_c"] = float(temp["mean_delta_c"])
        if "std_delta_c" in temp:
            flat["temp_std_delta_c"] = float(temp["std_delta_c"])
        if "min_delta_c" in temp:
            flat["temp_min_delta_c"] = float(temp["min_delta_c"])
        if "max_delta_c" in temp:
            flat["temp_max_delta_c"] = float(temp["max_delta_c"])

        eng = data.get("engine_degradation", {})
        if "min_power_multiplier" in eng:
            flat["engine_min_power_mult"] = float(eng["min_power_multiplier"])
        if "max_power_multiplier" in eng:
            flat["engine_max_power_mult"] = float(eng["max_power_multiplier"])
        if "min_fuel_flow_multiplier" in eng:
            flat["engine_min_fuel_mult"] = float(eng["min_fuel_flow_multiplier"])
        if "max_fuel_flow_multiplier" in eng:
            flat["engine_max_fuel_mult"] = float(eng["max_fuel_flow_multiplier"])

        batt = data.get("battery_uncertainty", {})
        if "min_resistance_multiplier" in batt:
            flat["battery_min_r_mult"] = float(batt["min_resistance_multiplier"])
        if "max_resistance_multiplier" in batt:
            flat["battery_max_r_mult"] = float(batt["max_resistance_multiplier"])
        if "max_initial_soc_offset_pct" in batt:
            flat["battery_max_soc_offset_pct"] = float(batt["max_initial_soc_offset_pct"])

        pwr = data.get("power_demand_uncertainty", {})
        if "min_multiplier" in pwr:
            flat["power_demand_min_mult"] = float(pwr["min_multiplier"])
        if "max_multiplier" in pwr:
            flat["power_demand_max_mult"] = float(pwr["max_multiplier"])

        mass = data.get("payload_mass_uncertainty", {})
        if "base_uav_mass_kg" in mass:
            flat["base_uav_mass_kg"] = float(mass["base_uav_mass_kg"])
        if "min_mass_delta_kg" in mass:
            flat["payload_mass_min_delta_kg"] = float(mass["min_mass_delta_kg"])
        if "max_mass_delta_kg" in mass:
            flat["payload_mass_max_delta_kg"] = float(mass["max_mass_delta_kg"])

        safety = data.get("safety_thresholds", {})
        if "min_fuel_reserve_liters" in safety:
            flat["min_fuel_reserve_liters"] = float(safety["min_fuel_reserve_liters"])
        if "min_battery_reserve_pct" in safety:
            flat["min_battery_reserve_pct"] = float(safety["min_battery_reserve_pct"])
        if "critical_battery_temp_c" in safety:
            flat["critical_battery_temp_c"] = float(safety["critical_battery_temp_c"])
        if "critical_engine_cht_c" in safety:
            flat["critical_engine_cht_c"] = float(safety["critical_engine_cht_c"])
        if "max_permitted_power_deficit_w" in safety:
            flat["max_permitted_power_deficit_w"] = float(safety["max_permitted_power_deficit_w"])

        stats = data.get("statistical_reporting", {})
        if "percentiles" in stats:
            flat["percentiles"] = [float(x) for x in stats["percentiles"]]
        if "confidence_level" in stats:
            flat["confidence_level"] = float(stats["confidence_level"])

        cfg = cls(**flat)
        cfg.validate()
        return cfg

    def validate(self):
        """Validates configuration parameters and enforces mathematical consistency."""
        if self.num_scenarios <= 0:
            raise ValueError(f"num_scenarios must be > 0, got {self.num_scenarios}")
        if self.simulation_timestep_s <= 0.0:
            raise ValueError(f"simulation_timestep_s must be > 0.0, got {self.simulation_timestep_s}")
        if self.max_simulation_duration_s <= 0.0:
            raise ValueError(f"max_simulation_duration_s must be > 0.0, got {self.max_simulation_duration_s}")
        if self.wind_max_speed_mps < 0.0:
            raise ValueError("wind_max_speed_mps cannot be negative")
        if self.temp_min_delta_c > self.temp_max_delta_c:
            raise ValueError("temp_min_delta_c cannot exceed temp_max_delta_c")
        if self.engine_min_power_mult > self.engine_max_power_mult:
            raise ValueError("engine_min_power_mult cannot exceed engine_max_power_mult")
        if self.battery_min_r_mult > self.battery_max_r_mult:
            raise ValueError("battery_min_r_mult cannot exceed battery_max_r_mult")
        if self.min_fuel_reserve_liters < 0.0:
            raise ValueError("min_fuel_reserve_liters cannot be negative")
        if not (0.0 < self.min_battery_reserve_pct < 100.0):
            raise ValueError("min_battery_reserve_pct must be between 0 and 100")
        if not (0.50 < self.confidence_level < 1.0):
            raise ValueError("confidence_level must be between 0.50 and 1.00")


class ScenarioGenerator:
    """
    Scenario generator providing documented deterministic and stochastic uncertainty realizations.
    """

    def __init__(self, config: Optional[Union[FastForwardConfig, Path, str, Dict[str, Any]]] = None):
        if config is None:
            default_path = Path(__file__).resolve().parent.parent / "configs" / "fast_forward_config.json"
            if default_path.exists():
                self.config = FastForwardConfig.from_json(default_path)
            else:
                self.config = FastForwardConfig()
                self.config.validate()
        elif isinstance(config, (str, Path)):
            self.config = FastForwardConfig.from_json(config)
        elif isinstance(config, dict):
            self.config = FastForwardConfig(**config)
            self.config.validate()
        elif isinstance(config, FastForwardConfig):
            config.validate()
            self.config = config
        else:
            raise TypeError(f"Invalid config type for ScenarioGenerator: {type(config)}")

    def generate_nominal_scenario(self) -> ScenarioParameters:
        """
        Generates the baseline zero-perturbation nominal flight scenario.
        Assumes standard calm ISA atmosphere and fresh baseline equipment.
        """
        return ScenarioParameters(
            scenario_id="SCENARIO_NOMINAL",
            scenario_type=ScenarioType.NOMINAL,
            wind_speed_mps=0.0,
            wind_direction_deg=0.0,
            ambient_temp_delta_c=0.0,
            engine_power_multiplier=1.00,
            engine_fuel_flow_multiplier=1.00,
            battery_resistance_multiplier=1.00,
            battery_initial_soc_offset_pct=0.0,
            propulsion_power_multiplier=1.00,
            payload_mass_delta_kg=0.0,
            description="Nominal baseline conditions: zero wind, standard ISA temp, healthy powertrain."
        )

    def generate_best_case_scenario(self, route_bearing_deg: float = 0.0) -> ScenarioParameters:
        """
        Generates best-case physical boundary scenario:
        - Tailwind aligned with route bearing (+5 m/s)
        - Cool ambient air (-5 deg C: denser air improves propeller thrust and thermal cooling)
        - Optimal engine performance (100% rated power, 98% nominal fuel consumption)
        - Low battery internal resistance (0.95x nominal)
        - Low aerodynamic drag / light payload (-20 kg)
        """
        # Tail-wind blows TOWARDS route_bearing_deg, meaning it comes FROM opposite direction
        tailwind_dir = (route_bearing_deg + 180.0) % 360.0

        return ScenarioParameters(
            scenario_id="SCENARIO_BEST_CASE",
            scenario_type=ScenarioType.BEST_CASE,
            wind_speed_mps=5.0,
            wind_direction_deg=round(tailwind_dir, 1),
            ambient_temp_delta_c=-5.0,
            engine_power_multiplier=1.00,
            engine_fuel_flow_multiplier=0.98,
            battery_resistance_multiplier=self.config.battery_min_r_mult,
            battery_initial_soc_offset_pct=1.5,
            propulsion_power_multiplier=self.config.power_demand_min_mult,
            payload_mass_delta_kg=self.config.payload_mass_min_delta_kg,
            description="Best-case physical bound: favorable tailwind, dense cooling air, peak efficiency."
        )

    def generate_worst_case_scenario(self, route_bearing_deg: float = 0.0) -> ScenarioParameters:
        """
        Generates worst-case physical boundary scenario:
        - Severe headwind directly opposing route bearing (up to max configured speed, e.g. 15 m/s)
        - Elevated ambient air temperature (+8 deg C: reduces air density and degrades cooling)
        - Degraded engine performance (85% rated power, +10% specific fuel consumption)
        - Elevated battery internal resistance (1.30x nominal: higher Joule heating, terminal sag)
        - High aerodynamic drag and heavy payload (+25 kg)
        """
        # Head-wind comes directly FROM the route bearing direction
        headwind_dir = route_bearing_deg % 360.0

        return ScenarioParameters(
            scenario_id="SCENARIO_WORST_CASE",
            scenario_type=ScenarioType.WORST_CASE,
            wind_speed_mps=self.config.wind_max_speed_mps,
            wind_direction_deg=round(headwind_dir, 1),
            ambient_temp_delta_c=self.config.temp_max_delta_c,
            engine_power_multiplier=self.config.engine_min_power_mult,
            engine_fuel_flow_multiplier=self.config.engine_max_fuel_mult,
            battery_resistance_multiplier=self.config.battery_max_r_mult,
            battery_initial_soc_offset_pct=-self.config.battery_max_soc_offset_pct,
            propulsion_power_multiplier=self.config.power_demand_max_mult,
            payload_mass_delta_kg=self.config.payload_mass_max_delta_kg,
            description="Worst-case bounded scenario: severe opposing headwind, hot air, degraded engine, elevated battery resistance."
        )

    def generate_monte_carlo_scenarios(
        self,
        count: Optional[int] = None,
        seed: Optional[int] = None
    ) -> List[ScenarioParameters]:
        """
        Generates reproducible Monte Carlo scenario ensemble sampled from documented distributions.

        Distribution Justifications:
        - Wind Speed: Truncated Gaussian (mu=0, sigma=4 m/s), bounded in [0, max_speed].
          Reflects typical atmospheric boundary layer wind statistics.
        - Wind Direction: Uniform in [0, 360 deg), as synoptic winds can blow from any direction.
        - Ambient Temp Delta: Truncated Gaussian (mu=0, sigma=2.5 deg C), bounded in [-8, +8 deg C].
          Reflects daily diurnal cycle and weather front variations.
        - Engine Power Derating: Uniform in [0.85, 1.00].
          Reflects random mechanical wear, turbo wear, and injector fouling.
        - Engine Fuel Flow: Uniform in [1.00, 1.10].
          Reflects specific fuel consumption deterioration.
        - Battery Resistance: Uniform in [0.95, 1.30].
          Reflects cell-to-cell variance, thermal distribution, and cycling aging.
        - Battery SoC Offset: Uniform in [-2.0%, +2.0%].
          Reflects Extended Kalman Filter state estimation uncertainty.
        - Drag / Power Demand: Uniform in [0.95, 1.08].
          Reflects airframe manufacturing tolerances, bugs/soot accumulation, and gusting.
        - Payload Mass Delta: Uniform in [-25 kg, +25 kg].
          Reflects payload avionics, sensor fitment, and fuel measurement uncertainty.
        """
        n = count if count is not None else self.config.num_scenarios
        actual_seed = seed if seed is not None else self.config.random_seed
        rng = random.Random(actual_seed)

        scenarios: List[ScenarioParameters] = []

        for i in range(1, n + 1):
            # 1. Wind speed & direction
            raw_wind = abs(rng.gauss(self.config.wind_mean_speed_mps, self.config.wind_std_speed_mps))
            w_speed = min(self.config.wind_max_speed_mps, raw_wind)
            w_dir = rng.uniform(0.0, 360.0)

            # 2. Ambient temp delta
            raw_temp = rng.gauss(self.config.temp_mean_delta_c, self.config.temp_std_delta_c)
            t_delta = max(self.config.temp_min_delta_c, min(self.config.temp_max_delta_c, raw_temp))

            # 3. Engine degradation
            eng_pwr = rng.uniform(self.config.engine_min_power_mult, self.config.engine_max_power_mult)
            eng_fuel = rng.uniform(self.config.engine_min_fuel_mult, self.config.engine_max_fuel_mult)

            # 4. Battery parameters
            batt_r = rng.uniform(self.config.battery_min_r_mult, self.config.battery_max_r_mult)
            soc_offset = rng.uniform(-self.config.battery_max_soc_offset_pct, self.config.battery_max_soc_offset_pct)

            # 5. Drag and mass
            pwr_mult = rng.uniform(self.config.power_demand_min_mult, self.config.power_demand_max_mult)
            mass_delta = rng.uniform(self.config.payload_mass_min_delta_kg, self.config.payload_mass_max_delta_kg)

            scenarios.append(ScenarioParameters(
                scenario_id=f"SCENARIO_MC_{i:03d}",
                scenario_type=ScenarioType.MONTE_CARLO,
                wind_speed_mps=round(w_speed, 2),
                wind_direction_deg=round(w_dir, 1),
                ambient_temp_delta_c=round(t_delta, 2),
                engine_power_multiplier=round(eng_pwr, 3),
                engine_fuel_flow_multiplier=round(eng_fuel, 3),
                battery_resistance_multiplier=round(batt_r, 3),
                battery_initial_soc_offset_pct=round(soc_offset, 2),
                propulsion_power_multiplier=round(pwr_mult, 3),
                payload_mass_delta_kg=round(mass_delta, 1),
                description=f"Monte Carlo scenario #{i} (Seed: {actual_seed})."
            ))

        return scenarios

    def generate_full_ensemble(
        self,
        route_bearing_deg: float = 0.0,
        include_deterministic_bounds: bool = True,
        seed: Optional[int] = None
    ) -> List[ScenarioParameters]:
        """
        Returns full scenario ensemble: Nominal, Best-Case, Worst-Case, plus Monte Carlo runs.
        """
        ensemble = []
        if include_deterministic_bounds:
            ensemble.append(self.generate_nominal_scenario())
            ensemble.append(self.generate_best_case_scenario(route_bearing_deg))
            ensemble.append(self.generate_worst_case_scenario(route_bearing_deg))

        mc_scenarios = self.generate_monte_carlo_scenarios(seed=seed)
        ensemble.extend(mc_scenarios)
        return ensemble
