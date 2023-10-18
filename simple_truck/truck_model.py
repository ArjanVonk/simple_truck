
class Truck:
    def __init__(self):
        self._speed = 0
        self._weight = 10000
        self._frontal_area = 8
        self._max_engine_force = 100_000
        self._throttle_percentage = 0.0
        self._is_started = False

    def _update_speed(self, force_forward, force_backward) -> None:
        force_resultant = force_forward - force_backward
        acceleration = force_resultant/self._weight
        self._speed += acceleration

    def _set_throttle_percentage(self, percentage: float) -> None:
        if percentage > 100:
            self._throttle_percentage = 100.0
        elif percentage < 0: 
            self._throttle_percentage = 0
        else:
            self._throttle_percentage = float(percentage)
        
    def _lower_throttle_percentage(self, percentage: float = 10.0) -> None:
        new_percentage = self.get_throttle_percentage() - percentage
        self._set_throttle_percentage(new_percentage)
    
    def _raise_throttle_percentage(self, percentage: float = 10.0) -> None:
        new_percentage = self.get_throttle_percentage() + percentage
        self._set_throttle_percentage(new_percentage)

    def start_truck(self) -> None:
        self._is_started = True

    def drive(self):
        if self._is_started:
            force_forward = self._throttle_percentage/100 * self._max_engine_force
        else:
            force_forward = 0
        force_backward = self._frontal_area * self._speed**2 
        self._update_speed(force_forward,force_backward)

    def turn_off_truck(self) -> None:
        self._is_started = False

    def get_status(self) -> str:
        return f"Truck speed: {self._speed:.1f}, Throttle: {self._throttle_percentage:.1f}, Started: {self._is_started}"
    
    def get_speed(self) -> float:
        return self._speed

    def get_throttle_percentage(self) -> float:
        return self._throttle_percentage

    def process_command(self, command_msg: str) -> None:
        message = command_msg.split("_",1)

        match message:
            case ["start"]:
                self.start_truck()
            case ["stop"]: 
                self.turn_off_truck()
            case ["throttle", *throttle_val]:
                self._set_throttle_percentage(float(throttle_val[0]))
            case ["raise", *raise_val]:
                self._raise_throttle_percentage(float(raise_val[0]))
            case ["lower", *lower_val]:
                self._lower_throttle_percentage(float(lower_val[0]))
            


    