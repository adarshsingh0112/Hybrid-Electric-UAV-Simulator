from scipy.optimize import minimize


class EnergyOptimizer:

    def __init__(self):

        self.minimum_soc = 20

    # -------------------------------------------------

    def optimize(
        self,
        required_power,
        battery_soc,
        fuel_remaining,
        mission_phase
    ):

        # Engine contribution for each mission phase
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

        target_engine = target_ratio * required_power

        # -------------------------------------------------

        def objective(x):

            engine_power = x[0]
            battery_power = x[1]

            # Fuel cost
            fuel_cost = engine_power * 0.05

            # Heavy penalty on battery usage
            battery_cost = battery_power * 0.12

            # Battery protection
            if battery_soc < 80:
                battery_cost += battery_power * 0.10

            if battery_soc < 60:
                battery_cost += battery_power * 0.15

            if battery_soc < 40:
                battery_cost += battery_power * 0.25

            # Keep engine near desired target
            balance_cost = (
                (engine_power - target_engine)
                / required_power
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
                x[0] + x[1] - required_power

            }

        ]

        bounds = [

            (0, required_power),

            (0, required_power)

        ]

        initial_guess = [

            target_engine,

            required_power - target_engine

        ]

        result = minimize(

            objective,

            initial_guess,

            method="SLSQP",

            bounds=bounds,

            constraints=constraints

        )

        engine_power = round(result.x[0], 2)

        battery_power = round(result.x[1], 2)

        return engine_power, battery_power