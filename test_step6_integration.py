import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from standalone_server import StandaloneEngineRuntime
from engine_service import EngineTwinRuntime

def test_integration():
    print("Testing StandaloneEngineRuntime with UAV Digital Twin...")
    s_runtime = StandaloneEngineRuntime()
    state1 = s_runtime.step(0.05)
    d1 = state1.to_dict()
    assert "uav_twin" in d1, "uav_twin missing from state dict!"
    uav1 = d1["uav_twin"]
    assert "battery" in uav1, "battery missing from uav_twin!"
    assert "powertrain" in uav1, "powertrain missing from uav_twin!"
    assert "mission" in uav1, "mission missing from uav_twin!"
    assert "quality_status" in uav1, "quality_status missing from uav_twin!"
    print("  -> StandaloneEngineRuntime telemetry payload verified successfully!")
    print(f"     Quality: {uav1['quality_status']} | Mode: {uav1['powertrain']['operating_mode']} | SoC: {uav1['battery']['soc_pct']}% | Progress: {uav1['mission']['mission_progress_pct']}%")

    print("\nTesting EngineTwinRuntime with UAV Digital Twin...")
    e_runtime = EngineTwinRuntime()
    state2 = e_runtime.step(0.05)
    d2 = state2.to_dict()
    assert "uav_twin" in d2, "uav_twin missing from state dict!"
    uav2 = d2["uav_twin"]
    assert "battery" in uav2, "battery missing from uav_twin!"
    assert "powertrain" in uav2, "powertrain missing from uav_twin!"
    assert "mission" in uav2, "mission missing from uav_twin!"
    assert "quality_status" in uav2, "quality_status missing from uav_twin!"
    print("  -> EngineTwinRuntime telemetry payload verified successfully!")
    print(f"     Quality: {uav2['quality_status']} | Mode: {uav2['powertrain']['operating_mode']} | SoC: {uav2['battery']['soc_pct']}% | Progress: {uav2['mission']['mission_progress_pct']}%")

    print("\n==========================================================")
    print("STEP 6 BACKEND & TELEMETRY INTEGRATION TEST PASSED! (100% CLEAN)")
    print("==========================================================")

if __name__ == "__main__":
    test_integration()
