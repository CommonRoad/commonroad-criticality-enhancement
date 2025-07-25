from pathlib import Path
from typing import List, Tuple

import numpy as np
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Scenario


def apply_update(target_vehicle: PlanningProblem, variable_type: str, delta: float, direction: str = "x") -> None:
    """
    Applies a delta update to a specified decision variable of a vehicle. This function is used for the gradient-based optimization approach.

    Parameters
    ----------
    target_vehicle : PlanningProblem
        The vehicle whose velocity or position is being adjusted. Can be used as DynamicObstacle for other vehicles in the future.

    variable_type : str
        The variable to update ("velocity" or "position").

    delta : float
        The amount to adjust the variable by.

    direction : str, optional
        The direction of the variable. Default is "x".

    Raises
    ------
    ValueError
        If the resulting velocity is non-positive or an unsupported variable type is provided.
    """
    if variable_type == "velocity":
        new_velocity = target_vehicle.initial_state.velocity + delta
        if new_velocity > 1:
            target_vehicle.initial_state.velocity = new_velocity
        else:
            print(f"Skipping update: velocity would become {new_velocity:.3f}, which is non-positive or too small.")
    elif variable_type == "position":
        x, y = target_vehicle.initial_state.position
        if direction == "x":
            new_pos = np.array([x + delta, y])
        else:
            new_pos = np.array([x, y + delta])
        target_vehicle.initial_state.position = new_pos
    else:
        raise ValueError(f"Unsupported variable type: {variable_type}")


def save_modified_scenario(scenario: Scenario, path: str, planning_problem_set: PlanningProblemSet) -> str:
    """
    Saves a modified CommonRoad scenario and planning problem set to a fixed XML file path. This function is used for the gradient-based optimization approach.

    Parameters
    ----------
    scenario : Scenario
        The modified CommonRoad scenario.

    path : str
        The path of the scenario.

    planning_problem_set : PlanningProblemSet
        The associated planning problem set.

    Returns
    -------
    str
        The relative file path to the saved scenario.
    """

    output_dir = Path(__file__).resolve().parent.parent / "scenarios"
    output_dir.mkdir(parents=True, exist_ok=True)  # ensure 'scenarios/' exists
    if "updated_gradient" not in path:
        path_obj = Path(path)
        path = path_obj.with_name(path_obj.stem + "_updated_gradient" + path_obj.suffix)
    writer = CommonRoadFileWriter(scenario=scenario, planning_problem_set=planning_problem_set, decimal_precision=10)
    writer.write_to_file(str(path), overwrite_existing_file=OverwriteExistingFile.ALWAYS)
    print(f"The new scenario was saved in {path}")
    return str(path)


def apply_variables_to_scenario(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    params: List[float],
    decision_variables: List[Tuple[str, str]],
    sa: bool,
) -> str:
    """
    Applies a set of variable updates (e.g. velocity or position) to a CommonRoad scenario and saves the result. This function is used for the BO and SA approaches.

    Each variable in `decision_variables` is updated with the corresponding value in `params`.
    Modifications are applied directly to the scenario's initial states, and the updated scenario is saved.

    Parameters
    ----------
    scenario : Scenario
        The CommonRoad scenario to be modified.

    planning_problem_set : PlanningProblemSet
        Set of planning problems (used to locate the 'ego' vehicle).

    params : List[float]
        List of new values to assign, one for each decision variable.

    decision_variables : List[Tuple[str, str]]
        List of (vehicle_id, variable_type) tuples, where:
        - vehicle_id (str): "ego" for the ego vehicle.
        - variable_type (str): Either "velocity" or "position".

    sa : bool
        if true, save the modified scenario with SA name, otherwise save the modified scenario with BO name.

    Returns
    -------
    str
        The path to the updated scenario file.
    """

    for (vehicle_id, variable_type), new_value in zip(decision_variables, params):
        if vehicle_id == "ego":
            vehicle = list(planning_problem_set.planning_problem_dict.values())[0]
        else:
            raise ValueError(f"Program supports only ego vehicle currently")

        init_state = vehicle.initial_state

        # Update the value
        if variable_type == "velocity":
            init_state.velocity = new_value
        elif variable_type == "x-position":
            x, y = init_state.position
            init_state.position = np.array([new_value, y])
        elif variable_type == "y-position":
            x, y = init_state.position
            init_state.position = np.array([x, new_value])
        else:
            print(f"Unknown variable type: {variable_type}")

    # Save the updated scenario
    output_dir = Path(__file__).resolve().parent.parent / "scenarios"
    output_dir.mkdir(parents=True, exist_ok=True)  # ensure 'scenarios/' exists
    if sa:
        temp_file = output_dir / "updated_scenario_sa.xml"
    else:
        temp_file = output_dir / "updated_scenario_bo.xml"
    writer = CommonRoadFileWriter(scenario, planning_problem_set)
    writer.write_to_file(str(temp_file), overwrite_existing_file=OverwriteExistingFile.ALWAYS)
    print(f"The new scenario was saved in {str(temp_file)}")
    return str(temp_file)
