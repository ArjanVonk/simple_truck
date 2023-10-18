from simple_truck.truck_model import Truck

class SimpleCommandProcessor:

    def process_string_command(self,command_msg: str, truck: Truck) -> None:
        message = command_msg.split("_",1)

        match message:
            case ["start"]:
                truck.start_truck()
            case ["stop"]: 
                truck.turn_off_truck()
            case ["throttle", *throttle_val]:
                truck.set_throttle_percentage(float(throttle_val[0]))
            case ["raise", *raise_val]:
                truck.raise_throttle_percentage(float(raise_val[0]))
            case ["lower", *lower_val]:
                truck.lower_throttle_percentage(float(lower_val[0]))
            