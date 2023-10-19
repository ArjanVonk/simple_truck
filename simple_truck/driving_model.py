from abc import ABC, abstractmethod

class SpeedCalculationModel(ABC):
    @abstractmethod
    def calculate_speed(self, current_speed: float, throttle_percentage: float) -> float:
        return 0.0

class EVSpeedCalculationModel(SpeedCalculationModel):
    """Speed calculation model for an EV that does not have ignition"""

    def __init__(self, engine_force: float = 100_000, frontal_area: float = 8.0, weight: float = 10000):
        """Speed calculation model of an EV

        Args:
            engine_force (float, optional): Maximum force delivered by engine. Defaults to 100_000.
            frontal_area (float, optional): Frontal area of truck in square meter. Defaults to 8.0.
            weight (float, optional): weight of truck. Defaults to 10000.
        """
        self._max_engine_force = engine_force
        self._frontal_area = frontal_area
        self._weight = weight

    def calculate_speed(self, current_speed: float, throttle_percentage: float) -> float:
        """Calculate the speed at the next time interval based on a basic physics model

        Args:
            current_speed (float): current speed of the object
            throttle_percentage (float): current throttle percentage of the motor of the object

        Returns:
            float: speed of the object at next timestep
        """
        force_forward = throttle_percentage/100 * self._max_engine_force
        force_backward = self._frontal_area * current_speed**2
        force_resultant = force_forward - force_backward
        acceleration = force_resultant/self._weight
        current_speed += acceleration 
        return current_speed

class TruckSpeedCalculationModel(SpeedCalculationModel):
    """Speed calculation model for a truck that contains the possibility to start the truck
    """

    def __init__(self, engine_force: float = 100_000, frontal_area: float = 8.0, weight: float = 10000, is_started: bool = False):
        """Speed calculation model of a truck

        Args:
            engine_force (float, optional): Maximum force delivered by engine. Defaults to 100_000.
            frontal_area (float, optional): Frontal area of truck in square meter. Defaults to 8.0.
            weight (float, optional): weight of truck. Defaults to 10000.
            is_started (bool, optional): ignition of truck. Defaults to False.
        """
        self._max_engine_force = engine_force
        self._frontal_area = frontal_area
        self._weight = weight
        self._is_started = is_started

    def calculate_speed(self, current_speed: float, throttle_percentage: float) -> float:
        """Calculate the speed at the next time interval based on a basic physics model

        Args:
            current_speed (float): current speed of the object
            throttle_percentage (float): current throttle percentage of the motor of the object

        Returns:
            float: speed of the object at next timestep
        """
        if self._is_started:
            force_forward = throttle_percentage/100 * self._max_engine_force
        else:
            force_forward = 0
        force_backward = self._frontal_area * current_speed**2
        force_resultant = force_forward - force_backward
        acceleration = force_resultant/self._weight
        current_speed += acceleration 
        return current_speed
    


    