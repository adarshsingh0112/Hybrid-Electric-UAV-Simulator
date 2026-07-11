class Motor:

    def __init__(self):

        self.name = "Electric Propulsion Motor"

        self.max_power = 250
        self.maximum_power = self.max_power

        self.efficiency = 0.95
        self.maximum_rpm = 12000

        self.running = False
        self.current_power = 0
        self.current_rpm = 0
        self.current_thrust = 0

    def start(self):

        self.running = True
        print("\n⚡ Electric Motor Started")

    def stop(self):

        self.running = False
        self.current_power = 0
        self.current_rpm = 0
        self.current_thrust = 0

        print("\n⚡ Electric Motor Stopped")

    def set_power(self, power):

        if power > self.max_power:
            power = self.max_power

        if power < 0:
            power = 0

        self.current_power = power

        self.current_rpm = int(
            (power / self.max_power) * self.maximum_rpm
        )

        self.current_thrust = round(power * 4.5, 2)

    def display_information(self):

        print("\n========== MOTOR ==========")
        print(f"Motor            : {self.name}")
        print(f"Maximum Power    : {self.max_power} kW")
        print(f"Current Power    : {self.current_power:.1f} kW")
        print(f"RPM              : {self.current_rpm}")
        print(f"Efficiency       : {self.efficiency}")
        print(f"Estimated Thrust : {self.current_thrust:.2f} N")
        print("===========================")

    def motor_status(self):

        return {
            "power": round(self.current_power, 2),
            "rpm": self.current_rpm,
            "thrust": self.current_thrust
        }