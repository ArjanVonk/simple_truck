from simple_truck.truck_model import Truck
import numpy as np


def test_startup():
    truck = Truck()
    assert not truck._is_started
    truck.start_truck()
    assert truck._is_started


def test_drive():
    truck = Truck()
    assert np.isclose(truck.get_speed(),0)
    for t in range(100):
        truck.drive()
        print(f"velocity at t= {t}: {truck.get_speed():.1f}")
        if t == 3: 
            truck.start_truck()
            truck._set_throttle_percentage(100)
            truck_speed_at_3 = truck.get_speed()
        if t == 35:
            truck.turn_off_truck()
            truck_speed_at_35 = truck.get_speed()
            assert truck_speed_at_35 > truck_speed_at_3
            print("turned off engine")
        if t == 40: 
            truck.start_truck()
            truck_speed_at_40 = truck.get_speed()
            assert truck_speed_at_40 < truck_speed_at_35
            print("turned on engine again")
        if t == 60: 
            truck._set_throttle_percentage(75)
            print("set throttle to 75")
 