"""
ENGINE-TWIN Simulation Package
Physics-Based Engine Modelling, Flight Profiles, Sensor Dynamics, and Fault Injection.
"""
from .flight_profile import FlightProfile, FlightState
from .mvem import MeanValueEngineModel, EngineState
from .sensors import SensorModel, SensorReadings
from .fault_injector import FaultInjector, FaultType, FaultConfig
from .battery import BatteryModel, BatteryConfig, BatteryState, BatteryTelemetry, BatteryChargeState
from .hybrid_powertrain import (
    HybridPowertrain,
    HybridPowertrainConfig,
    HybridOperatingMode,
    HybridControlMode,
    HybridPowertrainState,
    HybridPowertrainTelemetry,
    ElectricMachineModel,
    ElectricMachineState,
    PowerManagementController,
)

__all__ = [
    "FlightProfile",
    "FlightState",
    "MeanValueEngineModel",
    "EngineState",
    "SensorModel",
    "SensorReadings",
    "FaultInjector",
    "FaultType",
    "FaultConfig",
    "BatteryModel",
    "BatteryConfig",
    "BatteryState",
    "BatteryTelemetry",
    "BatteryChargeState",
    "HybridPowertrain",
    "HybridPowertrainConfig",
    "HybridOperatingMode",
    "HybridControlMode",
    "HybridPowertrainState",
    "HybridPowertrainTelemetry",
    "ElectricMachineModel",
    "ElectricMachineState",
    "PowerManagementController",
]
