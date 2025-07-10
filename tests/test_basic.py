import numpy as np
import pytest
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.scenario import Scenario, ScenarioID


@pytest.fixture
def scenario() -> Scenario:
    lanelet = Lanelet(np.array([[0, 2], [1, 2]]), np.array([[0, 1], [1, 1]]), np.array([[0, 0], [1, 0]]), 51)
    scenario = Scenario(0.2, ScenarioID())
    scenario.add_objects([lanelet])
    return scenario


def test_scenario_has_lanelet(scenario):
    assert len(scenario.lanelet_network.lanelets) == 1
    assert scenario.lanelet_network.find_lanelet_by_id(51) is not None
