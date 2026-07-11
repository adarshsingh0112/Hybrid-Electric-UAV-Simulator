from optimization.optimizer import EnergyOptimizer


class PowerManager:

    def __init__(self, battery, engine, motor):

        self.battery = battery
        self.engine = engine
        self.motor = motor

        # Create optimizer
        self.optimizer = EnergyOptimizer()

    # -------------------------------------------------

    def execute(self, mission_phase, required_power, duration):

        # Get optimum engine/battery power split
        engine_power, battery_power = self.optimizer.optimize(

            required_power=required_power,

            battery_soc=self.battery.soc,

            fuel_remaining=self.engine.fuel_remaining,

            mission_phase=mission_phase

        )

        # If fuel is empty use battery only
        if self.engine.fuel_remaining <= 0:

            print("\n⚠ Fuel Empty - Switching to Battery Only")

            engine_power = 0
            battery_power = required_power

        # Apply engine power
        self.engine.set_power(engine_power)
        engine_power = self.engine.current_power

        # Apply motor power
        self.motor.set_power(required_power)

        # Consume battery energy
        self.battery.consume_energy(
            battery_power,
            duration
        )

        # Consume fuel
        self.engine.consume_fuel(
            duration
        )

        # Display optimizer result
        print("\n========== ENERGY OPTIMIZER ==========")

        print(f"Mission Phase : {mission_phase}")

        print(f"Required Power: {required_power:.1f} kW")

        print(f"Engine Power  : {engine_power:.1f} kW")

        print(f"Battery Power : {battery_power:.1f} kW")

        print(f"Battery SoC   : {self.battery.soc:.1f}%")

        print(f"Fuel Left     : {self.engine.fuel_remaining:.2f} kg")

        print("======================================")

        return engine_power, battery_power

    # -------------------------------------------------

    def display_status(self):

        print("\n========== POWER MANAGER ==========")

        print(f"Battery SoC      : {self.battery.soc:.1f}%")
        print(f"Fuel Remaining   : {self.engine.fuel_remaining:.2f} kg")
        print(f"Engine Power     : {self.engine.current_power:.1f} kW")
        print(f"Motor RPM        : {self.motor.current_rpm}")
        print(f"Motor Thrust     : {self.motor.current_thrust:.2f} N")

        print("===================================")