import numpy as np
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.scenario import Scenario, ScenarioID


def test_scenario_has_lanelet():
    left_bound = np.array([[0, 2], [1, 2]])
    center_line = np.array([[0, 1], [1, 1]])
    right_bound = np.array([[0, 0], [1, 0]])

    # Create a Lanelet object with ID 51
    lanelet = Lanelet(left_bound, center_line, right_bound, 51)

    # Create a new Scenario object
    scenario = Scenario(0.2, ScenarioID())

    # Add the lanelet to the scenario
    scenario.add_objects([lanelet])

    # Assertions
    assert len(scenario.lanelet_network.lanelets) == 1
    assert scenario.lanelet_network.find_lanelet_by_id(51) is not None
