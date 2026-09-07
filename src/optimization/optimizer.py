from scipy.optimize import minimize


class EnergyOptimizer:

    def __init__(self, generator_efficiency=0.93, engine_idle_power=40.0, engine_max_power=300.0):

        self.minimum_soc = 20
        self.generator_efficiency = generator_efficiency
        self.engine_idle_power = engine_idle_power
        self.engine_max_power = engine_max_power

    # -------------------------------------------------

    def optimize(
        self,
        required_power,
        battery_soc,
        fuel_remaining,
        mission_phase
    ):

        # Engine contribution for each mission phase (electrical ratio)
        phase_ratio = {

            "Takeoff": 0.70,

            "Climb": 0.75,

            "Cruise": 0.82,

            "Loiter": 0.65,

            "Descent": 0.50,

            "Landing": 0.40

        }

        target_ratio = phase_ratio.get(mission_phase, 0.70)

        # Protect battery
        if battery_soc <= 60:

            target_ratio = max(target_ratio, 0.85)

        if battery_soc <= 40:

            target_ratio = max(target_ratio, 0.90)

        if battery_soc <= 25:

            target_ratio = 1.00

        # Fuel saving mode
        if fuel_remaining <= 30:

            target_ratio = max(target_ratio, 0.60)

        if fuel_remaining <= 0:
            return 0.0, required_power

        target_engine_elec = target_ratio * required_power
        target_engine_mech = min(
            self.engine_max_power,
            max(self.engine_idle_power, target_engine_elec / self.generator_efficiency)
        )

        # -------------------------------------------------

        def objective(x):

            engine_mech = x[0]
            battery_elec = x[1]

            # Fuel cost based on engine mechanical power
            fuel_cost = engine_mech * 0.05

            # Penalty on battery usage
            battery_cost = battery_elec * 0.12

            # Battery protection
            if battery_soc < 80:
                battery_cost += battery_elec * 0.10

            if battery_soc < 60:
                battery_cost += battery_elec * 0.15

            if battery_soc < 40:
                battery_cost += battery_elec * 0.25

            # Keep engine near desired target
            balance_cost = (
                (engine_mech - target_engine_mech)
                / max(1.0, required_power)
            ) ** 2

            return (

                fuel_cost

                + battery_cost

                + 40 * balance_cost

            )

        # -------------------------------------------------

        constraints = [

            {

                "type": "eq",

                "fun": lambda x:
                (x[0] * self.generator_efficiency) + x[1] - required_power

            }

        ]

        bounds = [

            (self.engine_idle_power, self.engine_max_power),

            (0, max(0.0, required_power))

        ]

        initial_guess = [

            target_engine_mech,

            max(0.0, required_power - (target_engine_mech * self.generator_efficiency))

        ]

        result = minimize(

            objective,

            initial_guess,

            method="SLSQP",

            bounds=bounds,

            constraints=constraints

        )

        if result.success:
            engine_mech = round(float(result.x[0]), 2)
            battery_elec = round(float(result.x[1]), 2)
        else:
            engine_mech = round(target_engine_mech, 2)
            battery_elec = round(max(0.0, required_power - (engine_mech * self.generator_efficiency)), 2)

        return engine_mech, battery_elec