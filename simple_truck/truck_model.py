
class Truck:
    def __init__(self):
        self._speed = 0
        self._weight = 10000
        self._frontal_area = 8
        self._engine_force = 100_000
        self._throttle_percentage = 0.0
        self._is_started = False

    def drive(self):
        if self._is_started:
            force_forward = self._throttle_percentage/100 * self._engine_force
        else:
            force_forward = 0
        force_backward = self._frontal_area * self._speed**2 
        self._update_speed(force_forward,force_backward)

    def _update_speed(self, force_forward, force_backward) -> None:
        force_resultant = force_forward - force_backward
        acceleration = force_resultant/self._weight
        self._speed += acceleration

    def start_truck(self) -> None:
        self._is_started = True

    def turn_off_truck(self) -> None:
        self._is_started = False

    def get_status(self) -> str:
        return f"Truck speed: {self._speed:.1f}, Throttle: {self._throttle_percentage:.1f}, Started: {self._is_started}"
    
    def get_speed(self) -> float:
        return self._speed

    def get_throttle_percentage(self) -> float:
        return self._throttle_percentage

    def set_throttle_percentage(self, percentage: float) -> None:
        self._throttle_percentage = float(percentage)
        
    def lower_throttle_percentage(self, percentage: float = 10.0) -> None:
        self._throttle_percentage -= percentage
        if self._throttle_percentage <= 0:
            self._throttle_percentage = 0
    