from abc import ABC, abstractmethod

class DrivingModel(ABC):
    @abstractmethod
    def calculate_speed(self, current_speed: float, throttle_percentage: float) -> float:
        return 0.0

class ElectricDrivingModel(DrivingModel):
    def __init__(self, engine_force: float = 100_000, frontal_area: float = 8.0, weight: float = 10000):
        self._max_engine_force = engine_force
        self._frontal_area = frontal_area
        self._weight = weight

    def calculate_speed(self, current_speed: float, throttle_percentage: float) -> float:
        force_forward = throttle_percentage/100 * self._max_engine_force
        force_backward = self._frontal_area * current_speed**2
        force_resultant = force_forward - force_backward
        acceleration = force_resultant/self._weight
        current_speed += acceleration 
        return current_speed

class TruckDrivingModel(DrivingModel):
    def __init__(self, engine_force: float = 100_000, frontal_area: float = 8.0, weight: float = 10000, is_started: bool = False):
        self._max_engine_force = engine_force
        self._frontal_area = frontal_area
        self._weight = weight
        self._is_started = is_started

    def calculate_speed(self, current_speed: float, throttle_percentage: float) -> float:
        if self._is_started:
            force_forward = throttle_percentage/100 * self._max_engine_force
        else:
            force_forward = 0
        force_backward = self._frontal_area * current_speed**2
        force_resultant = force_forward - force_backward
        acceleration = force_resultant/self._weight
        current_speed += acceleration 
        return current_speed
    


    