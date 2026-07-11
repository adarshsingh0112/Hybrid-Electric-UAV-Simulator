class Dashboard:

    def __init__(self):
        pass

    def bar(self, value, maximum, length=30):

        if maximum <= 0:
            return "-" * length

        filled = int((value / maximum) * length)
        filled = max(0, min(filled, length))

        return "#" * filled + "-" * (length - filled)

    def display(self, phase, mission_time, engine, battery, motor):

        print("\n" + "=" * 75)
        print("               HYBRID ELECTRIC UAV LIVE DASHBOARD")
        print("=" * 75)

        print(f"Mission Phase : {phase}")
        print(f"Mission Time  : {mission_time} min")
        print()

        print(
            f"Engine Power   : [{self.bar(engine.current_power, engine.max_power):30}] "
            f"{engine.current_power:7.1f} kW"
        )

        print(
            f"Battery SoC    : [{self.bar(battery.soc, 100):30}] "
            f"{battery.soc:7.1f} %"
        )

        print(
            f"Fuel Remaining : [{self.bar(engine.fuel_remaining, engine.fuel_capacity):30}] "
            f"{engine.fuel_remaining:7.2f} kg"
        )

        print(
            f"Motor RPM      : [{self.bar(motor.current_rpm, motor.maximum_rpm):30}] "
            f"{motor.current_rpm:7d} rpm"
        )

        print(
            f"Motor Thrust   : {motor.current_thrust:7.2f} N"
        )

        print("=" * 75)