import os
import sys
import unittest

import numpy as np
import ../enhance_criticality

sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "enhance_criticality"))
)
from enhance_criticality.reachability import (
    compute_drivable_slice_area,
    compute_full_drivable_area,
    differentiate_reachable_set_wrt_position,
    differentiate_reachable_set_wrt_velocity,
)


class DummyRegion:
    def __init__(self, lon_min, lon_max, lat_min, lat_max):
        self.p_lon_min = lon_min
        self.p_lon_max = lon_max
        self.p_lat_min = lat_min
        self.p_lat_max = lat_max


class DummyReachableSetInterface:
    def __init__(self):
        self.step_start = 0
        self.step_end = 2
        self.data = {
            0: [DummyRegion(0, 1, 0, 1), DummyRegion(0, 0.1, 0, 1)],
            1: [DummyRegion(0, 2, 0, 1)],
            2: [DummyRegion(1, 3, 2, 4), DummyRegion(0, 1, 0, 0.01)],
        }

    def drivable_area_at_step(self, step):
        return self.data[step]


class TestOptimizationFunctions(unittest.TestCase):
    def test_compute_drivable_slice_area(self):
        dummy_regions = [
            DummyRegion(0, 2, 0, 2),
            DummyRegion(2, 5, 1, 3),
        ]

        expected_area = 10
        result = compute_drivable_slice_area(dummy_regions)
        self.assertAlmostEqual(result, expected_area, msg="Drivable slice area incorrect.")

    def test_compute_full_drivable_area(self):
        expected_area = [1.1, 2, 4.01]
        result = compute_full_drivable_area(DummyReachableSetInterface())
        np.testing.assert_array_equal(result, expected_area)

    # def test_optimize_iteration_shapes(self):
    #     area_original = np.array([5.0, 6.0, 7.0])
    #     profile_matrix = np.array([
    #         [0.5, 0.3, 0.2],
    #         [0.1, 0.1, 0.1],
    #         [0.0, 0.0, 0.1],
    #     ])
    #     steps = 2
    #     result = optimize_iteration(area_original, profile_matrix, steps)
    #     self.assertEqual(len(result.value), profile_matrix.shape[0])

    # def test_differentiate_velocity_structure(self):
    #     # Mocking a ReachableSetInterface and vehicle
    #     class DummyReachInterface:
    #         step_start = 0
    #         step_end = 2
    #         def compute_reachable_sets(self):
    #             pass

    #     class DummyVehicle:
    #         class DummyState:
    #             def __init__(self):
    #                 self.velocity = 10
    #         initial_state = DummyState()

    #     dummy_reach = DummyReachInterface()
    #     dummy_vehicle = DummyVehicle()

    #     derivative = differentiate_reachable_set_wrt_velocity(dummy_reach, dummy_vehicle)
    #     self.assertEqual(len(derivative), 3)
    #     self.assertTrue(np.all(derivative >= -1))


if __name__ == "__main__":
    unittest.main()
