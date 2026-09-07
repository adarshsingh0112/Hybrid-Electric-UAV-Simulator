import pytest
import sys
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from environment.atmosphere import ISAAtmosphere


def test_sea_level_density_reasonable():
    """Verify sea level ISA density is approx 1.225 kg/m^3."""
    atmosphere = ISAAtmosphere()
    sea_level = atmosphere.get_conditions(0.0)

    assert pytest.approx(sea_level["density_kg_m3"], abs=1e-3) == 1.225
    assert pytest.approx(sea_level["temperature_k"], abs=1e-2) == 288.15
    assert pytest.approx(sea_level["pressure_pa"], abs=1.0) == 101325.0


def test_density_decreases_with_altitude():
    """Verify ISA air density decreases monotonically with altitude."""
    atmosphere = ISAAtmosphere()

    rho_0 = atmosphere.get_conditions(0.0)["density_kg_m3"]
    rho_1000 = atmosphere.get_conditions(1000.0)["density_kg_m3"]
    rho_5000 = atmosphere.get_conditions(5000.0)["density_kg_m3"]
    rho_10000 = atmosphere.get_conditions(10000.0)["density_kg_m3"]

    assert rho_0 > rho_1000 > rho_5000 > rho_10000, (
        f"Density failed to decrease monotonically: {rho_0}, {rho_1000}, {rho_5000}, {rho_10000}"
    )


def test_atmosphere_clamping_and_bounds():
    """Verify atmosphere model handles boundary conditions properly."""
    atmosphere = ISAAtmosphere()

    negative_alt = atmosphere.get_conditions(-500.0)
    assert negative_alt["density_kg_m3"] == 1.225

    high_alt = atmosphere.get_conditions(15000.0)
    assert high_alt["temperature_k"] > 0.0
    assert high_alt["density_kg_m3"] > 0.0
