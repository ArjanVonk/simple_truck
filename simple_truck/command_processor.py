from simple_truck.truck_model import Truck

class SimpleCommandProcessor:
    """The Command processor provides an interface to transform command messages to changes in the truck
    """

    def process_string_command(self,command_msg: str, truck: Truck) -> None:
        """Process a string command and perform actions with the truck accordingly

        Args:
            command_msg (str): command for the truck
            truck (Truck): truck that needs to do something based on command
        """
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
            