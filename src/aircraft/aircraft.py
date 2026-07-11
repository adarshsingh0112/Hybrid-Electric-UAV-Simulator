import json
from pathlib import Path


class Aircraft:
    def __init__(self):
        base_dir = Path(__file__).resolve().parent.parent.parent
        data_file = base_dir / "data" / "uav_specs.json"

        with open(data_file, "r") as file:
            specs = json.load(file)

        self.name = specs["uav_name"]
        self.mtow = specs["maximum_takeoff_weight_kg"]
        self.empty_weight = specs["empty_weight_kg"]
        self.payload = specs["payload_weight_kg"]
        self.fuel = specs["fuel_capacity_kg"]
        self.battery_capacity = specs["battery_capacity_kwh"]
        self.cruise_speed = specs["cruise_speed_kmph"]
        self.max_speed = specs["maximum_speed_kmph"]
        self.ceiling = specs["service_ceiling_m"]
        self.range = specs["maximum_range_km"]
        self.endurance = specs["target_endurance_hr"]
        self.wing_span = specs["wing_span_m"]
        self.wing_area = specs["wing_area_sq_m"]
        self.lift_drag = specs["lift_to_drag_ratio"]

    def display_information(self):
        print("\n===== Aircraft Specifications =====")
        print(f"Aircraft : {self.name}")
        print(f"MTOW : {self.mtow} kg")
        print(f"Empty Weight : {self.empty_weight} kg")
        print(f"Payload : {self.payload} kg")
        print(f"Fuel : {self.fuel} kg")
        print(f"Battery : {self.battery_capacity} kWh")
        print(f"Cruise Speed : {self.cruise_speed} km/h")
        print(f"Maximum Speed : {self.max_speed} km/h")
        print(f"Service Ceiling : {self.ceiling} m")
        print(f"Range : {self.range} km")
        print(f"Endurance : {self.endurance} hr")
        print(f"Wingspan : {self.wing_span} m")
        print(f"Wing Area : {self.wing_area} sq.m")
        print(f"Lift / Drag Ratio : {self.lift_drag}")
        print("==================================")