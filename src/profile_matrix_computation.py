from typing import Dict, List, Tuple

import numpy as np
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Scenario

from . import file_modification, reach_flow


def get_valid_perturbation_step(
    scenario: Scenario,
    scenario_path: str,
    planning_problem_set: PlanningProblemSet,
    vehicle: PlanningProblem,
    semantics: str,
    decision_variable: str,
    initial_step: float = 1.0,
    min_step: float = 0.05,
) -> float:
    """
    Finds the largest valid perturbation step that does not cause area computation to fail.

    Parameters
    ----------
    scenario : Scenario
        The CommonRoad scenario.

    scenario_path : str
        The path to the scenario.

    planning_problem_set : PlanningProblemSet
        The planning problem set.

    vehicle : PlanningProblem
        The vehicle whose velocity or position is being adjusted. Can be used as DynamicObstacle for other vehicles in the future.

    semantics : str
        Semantics to be passed to compute area.

    decision_variable : str
        The name of the decision variable.

    initial_step : float, optional
        Starting perturbation amount. Defaults to 1.0.

    min_step : float, optional
        Smallest allowed perturbation. If below this, return 0. Defaults to 0.05.

    Returns
    -------
    float
        A valid perturbation step that does not crash area computation, or 0.0 if none works.
    """

    original_velocity = vehicle.initial_state.velocity
    original_position = vehicle.initial_state.position
    step = initial_step / 2

    # Search for a valid step
    while step >= min_step:
        if decision_variable == "velocity":
            vehicle.initial_state.velocity = original_velocity + step
        elif decision_variable == "x-position":
            vehicle.initial_state.position = np.array([original_position[0] + step, original_position[1]])
        elif decision_variable == "y-position":
            vehicle.initial_state.position = np.array([original_position[0], original_position[1] + step])
        mod_path = file_modification.save_modified_scenario(scenario, scenario_path, planning_problem_set)

        # If step is valid, return it, else try with a smaller step
        try:
            _ = reach_flow.compute_drivable_area(mod_path, semantics=semantics)
            return step
        except Exception as e:
            print(f"Perturbation step {step:.5f} failed: {e}")
            step /= 2.0

    # If the step is too small, return 0
    print("All perturbation steps failed. Returning 0.")
    vehicle.initial_state.velocity = original_velocity
    vehicle.initial_state.position = original_position
    _ = file_modification.save_modified_scenario(scenario, scenario_path, planning_problem_set)
    return 0.0


def differentiate_reachable_set_wrt_velocity(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    scenario_path: str,
    step_start: int,
    step_end: int,
    vehicle: PlanningProblem,
    semantics: str,
) -> np.ndarray:
    """
    Computes the sensitivity of the drivable area with respect to a vehicle's initial velocity.
    The function perturbs the vehicle's initial velocity slightly and measures how the drivable
    area changes at each time step.

    Parameters
    ----------
    scenario : Scenario
        The CommonRoad scenario.

    planning_problem_set : PlanningProblemSet
        The planning problem set.

    scenario_path : str
        The path to the scenario.

    step_start : int
        The starting time step used for computation.

    step_end : int
        The ending time step used for computation.

    vehicle : PlanningProblem
        The vehicle whose velocity or position is being adjusted. Can be used as DynamicObstacle for other vehicles in the future.

    semantics : str
        Semantics to be passed to compute area.

    Returns
    -------
    np.ndarray
        An array representing the derivative of drivable area w.r.t. the vehicle's velocity.
    """

    original_velocity = vehicle.initial_state.velocity
    if original_velocity <= 0:
        raise ValueError(f"The original vehicle velocity must be positive, not {original_velocity}.")
    if original_velocity < 1:
        original_velocity = 1

    # Perturbation step
    h = original_velocity / 10

    derivative = np.array([-1.0 for _ in range(step_start, step_end + 1)])

    # Compute reachable area for original velocity
    area_original = reach_flow.compute_drivable_area(scenario_path, semantics=semantics)

    # Slightly increase velocity
    vehicle.initial_state.velocity += h

    # Save the modified scenario
    mod_scenario_path = file_modification.save_modified_scenario(scenario, scenario_path, planning_problem_set)

    # Recompute for the modified scenario
    try:
        area_changed_velocity = reach_flow.compute_drivable_area(mod_scenario_path, semantics=semantics)
    except Exception as e:
        print(f"Failed to compute area at changed velocity: {e} Trying smaller perturbations.")

        # If the perturbation leads to area 0, find a valid perturbation step
        vehicle.initial_state.velocity = original_velocity
        valid_perturbation = get_valid_perturbation_step(
            scenario, scenario_path, planning_problem_set, vehicle, semantics, "velocity", h
        )
        h = valid_perturbation
        if h == 0:
            return np.array([0.0 for _ in range(step_start, step_end + 1)])
        area_changed_velocity = reach_flow.compute_drivable_area(mod_scenario_path, semantics=semantics)

    # Derivative using h method
    for time_step in range(step_start, step_end + 1):
        derivative[time_step] = (area_changed_velocity[time_step] - area_original[time_step]) / h

    # Reset back to original velocity and save
    vehicle.initial_state.velocity = original_velocity
    _ = file_modification.save_modified_scenario(scenario, scenario_path, planning_problem_set)

    return derivative


def differentiate_reachable_set_wrt_position(
    x: bool,
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    scenario_path: str,
    step_start: int,
    step_end: int,
    vehicle: PlanningProblem,
    semantics: str,
) -> np.ndarray:
    """
    Computes the sensitivity of the drivable area with respect to a vehicle's initial position.
    This function perturbs the coordinate of the vehicle's initial position and computes
    the change in reachable area at each time step.

    Parameters
    ----------
    x : bool
        If true, update in x-direction; otherwise, in y-direction.

    scenario : Scenario
        The CommonRoad scenario.

    planning_problem_set : PlanningProblemSet
        The planning problem set.

    scenario_path : str
        The path to the scenario.

    step_start : int
        Start time step of reachability analysis.

    step_end : int
        End time step of reachability analysis.

    vehicle : PlanningProblem
        The vehicle whose velocity or position is being adjusted. Can be used as DynamicObstacle for other vehicles in the future.

    semantics : str
        Semantics to be passed to compute area.

    Returns
    -------
    np.ndarray
        An array of numerical derivatives representing sensitivity of drivable area to position.
    """

    original_position = vehicle.initial_state.position

    # Define a perturbation step
    delta_pos = 1

    derivative = np.array([-1.0 for _ in range(step_start, step_end + 1)])

    # Compute reachable area for original position
    area_original = reach_flow.compute_drivable_area(scenario_path, semantics=semantics)

    # Apply the perturbation
    if x:
        vehicle.initial_state.position = np.array([original_position[0] + delta_pos, original_position[1]])
    else:
        vehicle.initial_state.position = np.array([original_position[0], original_position[1] + delta_pos])

    # Save the modified scenario
    mod_scenario_path = file_modification.save_modified_scenario(scenario, scenario_path, planning_problem_set)

    # Recompute for the modified scenario
    try:
        area_changed_position = reach_flow.compute_drivable_area(mod_scenario_path, semantics=semantics)
    except Exception as e:
        print(f"Failed to compute area at changed position: {e} Trying smaller perturbations.")

        # If the perturbation leads to area 0, find a valid perturbation step
        vehicle.initial_state.position = original_position
        if x:
            valid_perturbation = get_valid_perturbation_step(
                scenario, scenario_path, planning_problem_set, vehicle, semantics, "x-position"
            )
        else:
            valid_perturbation = get_valid_perturbation_step(
                scenario, scenario_path, planning_problem_set, vehicle, semantics, "y-position"
            )
        delta_pos = valid_perturbation
        if delta_pos == 0:
            return np.array([0.0 for _ in range(step_start, step_end + 1)])
        area_changed_position = reach_flow.compute_drivable_area(mod_scenario_path, semantics=semantics)

    print("original area: ", area_original)
    print("modified area: ", area_changed_position)

    # Derivative using h method
    for time_step in range(step_start, step_end + 1):
        derivative[time_step] = (area_changed_position[time_step] - area_original[time_step]) / delta_pos

    # Reset back to original position and save
    vehicle.initial_state.position = original_position
    _ = file_modification.save_modified_scenario(scenario, scenario_path, planning_problem_set)

    return derivative


def get_profile_matrix(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    scenario_path: str,
    step_start: int,
    step_end: int,
    decision_variables: List[Tuple[str, str]],
    semantics: str,
) -> Tuple[np.ndarray, Dict[Tuple[str, str], int]]:
    """
    Constructs a profile matrix showing the sensitivity of drivable area to specified decision variables.

    Each decision variable is a tuple consisting of a vehicle ID and a variable type.
    The function perturbs the specified variable (velocity or position) for the given vehicle,
    computes the effect on the reachable drivable area over time, and stores it as a row in the matrix.
    Accepts "velocity", "x-position", "y-position", or "position" (for both x and y).

    Parameters
    ----------
    scenario : Scenario
        The CommonRoad scenario.

    planning_problem_set : PlanningProblemSet
        The planning problem set.

    scenario_path : str
        The path to the scenario.

    step_start : int
        Start time step of reachability analysis.

    step_end : int
        End time step of reachability analysis.

    decision_variables : List[Tuple[str, str]]
        List of tuples (vehicle_id, variable_type), where:
        - vehicle_id (str): "ego".
        - variable_type (str): "velocity", "position", etc.

    semantics : str
        Semantics to be passed to compute area.

    Returns
    -------
    profile_matrix : np.ndarray
        Array where each row is a derivative profile over time.
    profile_index_map : dict
        Maps (vehicle_id, variable_type) to its row index in the matrix.
    """

    result = []
    profile_index_map = {}

    row_idx = 0  # Keeps track of the profile_matrix row index

    # Compute derivatives for each given decision variable
    for vehicle_id, variable_type in decision_variables:
        if vehicle_id == "ego":
            vehicle = list(planning_problem_set.planning_problem_dict.values())[0]
        else:
            raise ValueError(f"Program supports only ego vehicle currently")

        if vehicle is None:
            print(f"Warning: Vehicle {vehicle_id} not found.")
            continue

        # Velocity derivative
        if variable_type == "velocity":
            deriv = differentiate_reachable_set_wrt_velocity(
                scenario, planning_problem_set, scenario_path, step_start, step_end, vehicle, semantics
            )
            result.append(deriv)
            profile_index_map[(vehicle_id, "velocity")] = row_idx
            row_idx += 1

        # X position derivative
        elif variable_type == "x-position":
            deriv = differentiate_reachable_set_wrt_position(
                x=True,
                scenario=scenario,
                planning_problem_set=planning_problem_set,
                scenario_path=scenario_path,
                step_start=step_start,
                step_end=step_end,
                vehicle=vehicle,
                semantics=semantics,
            )
            result.append(deriv)
            profile_index_map[(vehicle_id, "x-position")] = row_idx
            row_idx += 1

        # Y position derivative
        elif variable_type == "y-position":
            deriv = differentiate_reachable_set_wrt_position(
                x=False,
                scenario=scenario,
                planning_problem_set=planning_problem_set,
                scenario_path=scenario_path,
                step_start=step_start,
                step_end=step_end,
                vehicle=vehicle,
                semantics=semantics,
            )
            result.append(deriv)
            profile_index_map[(vehicle_id, "y-position")] = row_idx
            row_idx += 1
        else:
            print(f"Warning: Unknown variable type: {variable_type}")
            continue

    # Transform List to matrix
    profile_matrix = np.vstack(result)
    return profile_matrix, profile_index_map
