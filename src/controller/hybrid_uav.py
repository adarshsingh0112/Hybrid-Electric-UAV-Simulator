from aircraft.aircraft import Aircraft
from battery.battery import Battery
from mission.mission import Mission
from engine.engine import Engine
from motor.motor import Motor
from power_manager.power_manager import PowerManager
from mission.simulator import MissionSimulator


class HybridUAV:

    def __init__(self):

        self.aircraft = Aircraft()
        self.battery = Battery()
        self.mission = Mission()
        self.engine = Engine()
        self.motor = Motor()

        # Power Manager
        self.power_manager = PowerManager(
            self.battery,
            self.engine,
            self.motor
        )
        self.simulator = MissionSimulator(
            self.mission,
            self.power_manager
        )

    def initialize(self):

        print("\n========== INITIALIZING UAV ==========")

        self.aircraft.display_information()
        self.battery.display_information()
        self.engine.display_information()
        self.motor.display_information()
        self.mission.display_mission()

        print("\nInitialization Successful")

    def startup(self):
        print("\n========== STARTUP ==========")
        self.engine.start()
        self.motor.start()
        self.power_manager.display_status()

    def shutdown(self):

        print("\n========== SHUTDOWN ==========")

        self.motor.stop()
        self.engine.stop()

    def battery_test(self):

        print("\n========== BATTERY TEST ==========")

        print("\nDischarging Battery by 15%...")
        self.battery.discharge(15)
        self.battery.display_information()

        print("\nCharging Battery by 10%...")
        self.battery.charge(10)
        self.battery.display_information()

    def fly(self):
        self.startup()
        self.simulator.run()
        self.shutdown()