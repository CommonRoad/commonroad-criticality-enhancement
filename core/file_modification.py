import os
from typing import List, Tuple

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario


def save_modified_scenario(scenario: Scenario, planning_problem_set: PlanningProblemSet) -> str:
    """
    Saves a modified CommonRoad scenario and planning problem set to a fixed XML file path.

    Parameters:
    - scenario (Scenario): The modified CommonRoad scenario.
    - planning_problem_set (PlanningProblemSet): The associated planning problem set.

    Returns:
    - str: The relative file path to the saved scenario.
    """
    temp_file = os.path.join("scenarios", "modified_scenario.xml")
    writer = CommonRoadFileWriter(scenario, planning_problem_set)
    writer.write_to_file(temp_file, overwrite_existing_file=OverwriteExistingFile.ALWAYS)
    print("The new scenario was saved in modified_scenario.xml")
    return "scenarios/modified_scenario.xml"


def apply_variables_to_scenario(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    params: List[float],
    decision_variables: List[Tuple[str, str]],
) -> str:
    """
    Applies a set of variable updates (e.g. velocity or position) to a CommonRoad scenario and saves the result.

    Each variable in `decision_variables` is updated with the corresponding value in `params`.
    Modifications are applied directly to the scenario's initial states (in-place), and the updated scenario is saved.

    Parameters:
    - scenario (Scenario): The CommonRoad scenario object to be modified.
    - planning_problem_set (PlanningProblemSet): Set of planning problems (used to locate the 'ego' vehicle).
    - params (List[float]): List of new values to assign, one for each decision variable.
    - decision_variables (List[Tuple[str, str]]): List of (vehicle_id, variable_type) tuples, where:
        - vehicle_id (str): "ego" for the ego vehicle, or the stringified integer ID of a dynamic obstacle.
        - variable_type (str): Either "velocity" or "position".

    Returns:
    - str: The path to the updated scenario file (e.g. "scenarios/updated_scenario.xml").
    """
    for (vehicle_id, variable_type), new_value in zip(decision_variables, params):
        if vehicle_id == "ego":
            vehicle = list(planning_problem_set.planning_problem_dict.values())[0]
        else:
            try:
                vehicle_id = int(vehicle_id)
            except ValueError:
                print(f"Invalid vehicle ID: {vehicle_id}")
                continue

            vehicle = next(
                (v for v in scenario.dynamic_obstacles if v.obstacle_id == vehicle_id), None
            )
            if vehicle is None:
                print(f"Vehicle {vehicle_id} not found in dynamic obstacles.")
                continue

        init_state = vehicle.initial_state

        # Update the value
        if variable_type == "velocity":
            init_state.velocity = new_value
        elif variable_type == "position":
            x, y = init_state.position
            init_state.position = (new_value, y)
        else:
            print(f"Unknown variable type: {variable_type}")

    temp_file = os.path.join("scenarios", "updated_scenario.xml")
    writer = CommonRoadFileWriter(scenario, planning_problem_set)
    writer.write_to_file(temp_file, overwrite_existing_file=OverwriteExistingFile.ALWAYS)
    print("The new scenario was saved in updated_scenario.xml")
    return "scenarios/updated_scenario.xml"
