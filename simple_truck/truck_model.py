from abc import ABC, abstractmethod
from simple_truck.driving_model import TruckDrivingModel


class Truck:
    """The Truck class contains a simple truck with a driving model
    """
    def __init__(self):
        self._speed = 0
        self._max_engine_force= 100_000
        self._frontal_area = 8
        self._weight = 10_000
        self._is_started = False
        self._driving_model = TruckDrivingModel()
        self._update_driving_model()
        self._throttle_percentage = 0.0

    def _update_driving_model(self):
        self._driving_model = TruckDrivingModel(engine_force=self._max_engine_force,
                                               frontal_area=self._frontal_area,
                                               weight=self._weight,
                                               is_started=self._is_started)

    def set_throttle_percentage(self, percentage: float) -> None:
        if percentage > 100:
            self._throttle_percentage = 100.0
        elif percentage < 0: 
            self._throttle_percentage = 0
        else:
            self._throttle_percentage = float(percentage)
        
    def lower_throttle_percentage(self, percentage: float = 10.0) -> None:
        new_percentage = self.get_throttle_percentage() - percentage
        self.set_throttle_percentage(new_percentage)
    
    def raise_throttle_percentage(self, percentage: float = 10.0) -> None:
        new_percentage = self.get_throttle_percentage() + percentage
        self.set_throttle_percentage(new_percentage)

    def start_truck(self) -> None:
        self._is_started = True
        self._update_driving_model()

    def drive(self):
        self._speed = self._driving_model.calculate_speed(self._speed, self._throttle_percentage)

    def turn_off_truck(self) -> None:
        self._is_started = False
        self._update_driving_model()

    def get_status(self) -> str:
        return f"Truck speed: {self._speed:.1f}, Throttle: {self._throttle_percentage:.1f}, Started: {self._is_started}"
    
    def get_speed(self) -> float:
        return self._speed

    def get_throttle_percentage(self) -> float:
        return self._throttle_percentage