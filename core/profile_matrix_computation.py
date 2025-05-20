from typing import Dict, List, Tuple

import file_modification
import numpy as np
import reach_flow
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario


def differentiate_reachable_set_wrt_velocity(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    scenario_path: str,
    step_start: int,
    step_end: int,
    vehicle: DynamicObstacle,
) -> np.ndarray:
    """
    Computes the sensitivity of the drivable area with respect to a vehicle's initial velocity.
    The function perturbs the vehicle's initial velocity slightly and measures how the drivable
    area changes at each time step.

    Parameters:
    - scenario (Scenario):The CommonRoad scenario containing map and obstacle data.
    - planning_problem_set (PlanningProblemSet): The planning problem set.
    - scenario_path (str): The path to the scenario.
    - step_start (int): The starting time step used for computation.
    - step_end (int): The ending time step used for computation.
    - vehicle (DynamicObstacle): The vehicle whose velocity will be perturbed.

    Returns:
    - np.ndarray: An array representing the derivative of drivable area w.r.t. the vehicle's velocity.
    """

    original_velocity = vehicle.initial_state.velocity
    if original_velocity <= 0:
        raise ValueError(f"Vehicle velocity must be positive, not {original_velocity}.")
    if original_velocity < 1:
        original_velocity = 1
    h = original_velocity / 10

    derivative = np.array([-1.0 for i in range(step_start, step_end + 1)])

    # Compute reachable area for original velocity
    area_original = reach_flow.compute_drivable_area(scenario_path)

    # Slightly increase velocity
    vehicle.initial_state.velocity += h

    # Save the modified scenario
    mod_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)

    # Recompute for the modified scenario
    area_changed_velocity = reach_flow.compute_drivable_area(mod_scenario_path)

    # Derivative using h method
    for time_step in range(step_start, step_end + 1):
        derivative[time_step] = (area_changed_velocity[time_step] - area_original[time_step]) / h

    vehicle.initial_state.velocity = original_velocity

    return derivative


def differentiate_reachable_set_wrt_position(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    scenario_path: str,
    step_start: int,
    step_end: int,
    vehicle: DynamicObstacle,
    scenario_max_time: int,
) -> np.ndarray:
    """
    Computes the sensitivity of the drivable area with respect to a vehicle's initial x-position.
    This function perturbs the x-coordinate of the vehicle's initial position and computes
    the change in reachable area at each time step.

    Parameters:
    - scenario (Scenario): The CommonRoad scenario.
    - planning_problem_set (PlanningProblemSet): The planning problem set.
    - scenario_path (str): The path to the scenario.
    - step_start (int): Start time step of reachability analysis.
    - step_end (int): End time step of reachability analysis.
    - vehicle (DynamicObstacle): The vehicle whose x-position will be perturbed.
    - scenario_max_time (int): Maximum time span for the scenario.

    Returns:
    - np.ndarray: An array of numerical derivatives representing sensitivity of drivable area to position.
    """

    original_position = vehicle.initial_state.position

    delta_x = int(np.ceil(scenario_max_time / 10))
    delta_x = max(delta_x, 1)

    derivative = np.array([-1.0 for i in range(step_start, step_end + 1)])

    # Compute reachable area for original position
    area_original = reach_flow.compute_drivable_area(scenario_path)

    # Slightly increase velocity
    vehicle.initial_state.position = (
        vehicle.initial_state.position[0] + delta_x,
        vehicle.initial_state.position[1],
    )
    print("original position: ", original_position)
    print("modified position: ", vehicle.initial_state.position)

    # Save the modified scenario
    mod_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)

    # Recompute for the modified scenario
    area_changed_position = reach_flow.compute_drivable_area(mod_scenario_path)

    print("original area: ", area_original)
    print("modified area: ", area_changed_position)

    # Derivative using h method
    for time_step in range(step_start, step_end + 1):
        derivative[time_step] = (
            area_changed_position[time_step] - area_original[time_step]
        ) / delta_x

    vehicle.initial_state.position = original_position

    return derivative


def get_profile_matrix(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    scenario_path: str,
    step_start: int,
    step_end: int,
    decision_variables: List[Tuple[str, str]],
) -> Tuple[np.ndarray, Dict[Tuple[str, str], int]]:
    """
    Constructs a profile matrix showing the sensitivity of drivable area to specified decision variables.

    Each decision variable is a tuple consisting of a vehicle ID and a variable type.
    The function perturbs the specified variable (velocity or position) for the given vehicle,
    computes the effect on the reachable drivable area over time, and stores it as a row in the matrix.

    Parameters:
    - scenario (Scenario): The CommonRoad scenario.
    - planning_problem_set (PlanningProblemSet): The planning problem set.
    - scenario_path (str): The path to the scenario.
    - step_start (int): Start time step of reachability analysis.
    - step_end (int): End time step of reachability analysis.
    - decision_variables (List[Tuple[str, str]]): List of tuples of the form (vehicle_id, variable_type), where:
        - vehicle_id: a string, e.g., "ego" or "1"
        - variable_type: either "velocity" or "position"

    Returns:
    - profile_matrix: An array where each row is a derivative profile over time.
    - profile_index_map: A dictionary mapping (vehicle_id, variable_type) to its row index in the matrix.
    """

    result = []
    profile_index_map = {}
    scenario_max_time = step_end - step_start + 1

    row_idx = 0  # Keeps track of the profile_matrix row index

    # Compute derivatives for each given decision variable
    for vehicle_id, variable_type in decision_variables:
        if vehicle_id == "ego":
            vehicle = list(planning_problem_set.planning_problem_dict.values())[0]
        else:
            vehicle = next(
                (v for v in scenario.dynamic_obstacles if v.obstacle_id == vehicle_id), None
            )

        if vehicle is None:
            print(f"Warning: Vehicle {vehicle_id} not found.")
            continue

        if variable_type == "velocity":
            deriv = differentiate_reachable_set_wrt_velocity(
                scenario, planning_problem_set, scenario_path, step_start, step_end, vehicle
            )
        elif variable_type == "position":
            deriv = differentiate_reachable_set_wrt_position(
                scenario,
                planning_problem_set,
                scenario_path,
                step_start,
                step_end,
                vehicle,
                scenario_max_time,
            )
        else:
            print(f"Warning: Unknown variable type: {variable_type}")
            continue

        result.append(deriv)
        profile_index_map[(vehicle_id, variable_type)] = row_idx
        row_idx += 1

    # Transform List to matrix
    profile_matrix = np.vstack(result)
    return profile_matrix, profile_index_map
