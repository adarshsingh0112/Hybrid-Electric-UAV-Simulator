from optimization.optimizer import EnergyOptimizer


class PowerManager:

    def __init__(self, battery, engine, motor):

        self.battery = battery
        self.engine = engine
        self.motor = motor

        # Pass engine specs to optimizer
        gen_eff = getattr(engine, "generator_efficiency", 0.93)
        idle_pwr = getattr(engine, "idle_power", 40.0)
        max_pwr = getattr(engine, "max_power", 300.0)

        self.optimizer = EnergyOptimizer(
            generator_efficiency=gen_eff,
            engine_idle_power=idle_pwr,
            engine_max_power=max_pwr
        )

    # -------------------------------------------------

    def execute(self, mission_phase, required_power, duration, is_seconds: bool = False, verbose: bool = True):

        # 1. Obtain target split from optimizer
        engine_mech_target, battery_elec_target = self.optimizer.optimize(
            required_power=required_power,
            battery_soc=self.battery.soc,
            fuel_remaining=self.engine.fuel_remaining,
            mission_phase=mission_phase
        )

        # 2. Check fuel status
        if self.engine.fuel_remaining <= 0:
            if verbose:
                print("\n[WARN] Fuel Empty - Switching to Battery Only")
            engine_mech_target = 0.0

        # 3. Apply engine mechanical power target
        self.engine.set_power(engine_mech_target)
        engine_mech = self.engine.current_power
        engine_elec = self.engine.get_electrical_power()

        # 4. Handle electrical bus balance and excess generator power
        power_deficit = 0.0

        if engine_elec >= required_power:
            # Generator produces excess power -> supply bus demand first
            excess_elec = engine_elec - required_power

            # Query battery charge acceptance capability
            batt_max_chg = self.battery.get_available_charge_power()
            chg_power_elec = min(excess_elec, batt_max_chg)

            unabsorbed_elec = excess_elec - chg_power_elec
            if unabsorbed_elec > 1e-4:
                # Throttle engine down so no energy is created or silently dumped
                needed_gen_elec = required_power + chg_power_elec
                needed_engine_mech = needed_gen_elec / self.engine.generator_efficiency
                self.engine.set_power(needed_engine_mech)

                engine_mech = self.engine.current_power
                engine_elec = self.engine.get_electrical_power()

            # Battery power is negative during charging
            battery_elec = -chg_power_elec
            self.battery.recharge(chg_power_elec, duration, is_seconds=is_seconds)

        else:
            # Generator cannot meet total demand alone -> battery must discharge
            needed_battery_elec = required_power - engine_elec
            batt_avail_dis = self.battery.get_available_discharge_power()

            if needed_battery_elec > batt_avail_dis:
                power_deficit = needed_battery_elec - batt_avail_dis
                battery_elec = batt_avail_dis
                if verbose:
                    print(
                        f"\n[WARN] POWER DEFICIT: Bus Required {required_power:.1f} kW, "
                        f"Available {engine_elec + battery_elec:.1f} kW. Deficit: {power_deficit:.1f} kW"
                    )
            else:
                battery_elec = needed_battery_elec

            delivered_batt = self.battery.consume_energy(battery_elec, duration, is_seconds=is_seconds)
            # Use actual delivered power if battery energy was exhausted
            if battery_elec > 0 and delivered_batt < battery_elec:
                actual_shortfall = battery_elec - delivered_batt
                battery_elec = delivered_batt
                power_deficit += actual_shortfall

        # 5. Total electrical power supplied to DC bus
        supplied_bus_elec = engine_elec + battery_elec

        # 6. Apply electrical power to motor and consume fuel
        self.motor.set_power(supplied_bus_elec)
        self.engine.consume_fuel(duration, is_seconds=is_seconds)

        self.last_power_deficit = power_deficit
        self.last_is_power_limited = (power_deficit > 1e-4)

        # 7. Complete 7-domain power breakdown:
        motor_shaft_mech = self.motor.get_shaft_power()

        # 8. Display optimizer & power balance results if verbose
        if verbose:
            print("\n========== ENERGY OPTIMIZER ==========")
            print(f"Mission Phase : {mission_phase}")
            print(f"DC Bus Required: {required_power:.1f} kW")
            print(f"Engine Mech Pwr: {engine_mech:.1f} kW")
            print(f"Gen Elec Pwr   : {engine_elec:.1f} kW (Limit: {self.engine.generator_max_elec_kw:.1f} kW, Sat: {self.engine.is_generator_saturated()})")
            print(f"Battery Elec Pwr: {battery_elec:.1f} kW (+Dis/-Chg)")
            print(f"Motor Elec In  : {self.motor.current_elec_power:.1f} kW (Sat: {self.motor.is_motor_saturated()})")
            print(f"Motor Shaft Out: {motor_shaft_mech:.1f} kW")
            if power_deficit > 1e-4:
                print(f"Power Deficit  : {power_deficit:.1f} kW")
            print(f"Battery SoC    : {self.battery.soc:.1f}% (Reserve: {self.battery.min_soc:.1f}%)")
            print(f"Fuel Left      : {self.engine.fuel_remaining:.2f} kg")
            print("======================================")

        return engine_elec, battery_elec

    # -------------------------------------------------

    def display_status(self):

        print("\n========== POWER MANAGER ==========")

        print(f"Battery SoC      : {self.battery.soc:.1f}%")
        print(f"Fuel Remaining   : {self.engine.fuel_remaining:.2f} kg")
        print(f"Engine Mech Power: {self.engine.current_power:.1f} kW")
        print(f"Engine Elec Power: {self.engine.get_electrical_power():.1f} kW")
        print(f"Motor RPM        : {self.motor.current_rpm}")
        print(f"Motor Thrust     : {self.motor.current_thrust:.2f} N")

        print("===================================")