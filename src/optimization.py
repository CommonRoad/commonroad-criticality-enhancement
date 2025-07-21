import warnings
from typing import List, Tuple

import cvxpy as cv
import numpy as np
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from numpy import ndarray

import file_modification
import profile_matrix_computation
import reach_flow


def optimize_iteration(
    area_original: np.ndarray,
    profile_matrix: np.ndarray,
    step_end: int,
    lower_bounds: np.ndarray,
    upper_bounds: np.ndarray,
    step_start_opt: int = 6,
    a_ref_input: float = 1.0,
) -> cv.Variable:
    """
    Solves a quadratic program (QP) to compute optimal adjustments to decision variables.

    Parameters
    ----------
    area_original : np.ndarray
        The original drivable area over time.

    profile_matrix : np.ndarray
        Sensitivity matrix showing how each decision variable affects the area.

    step_end : int
        Last time step.

    lower_bounds : np.ndarray
        Constraints to regulate the step in an optimization iteration.

    upper_bounds : np.ndarray
        Constraints to regulate the step in an optimization iteration.

    step_start_opt : int, optional
        Time step to start the optimization, as initial time steps have very small areas. Default is 6.

    a_ref_input : float, optional
        Scalar to modify the area reference target. Default is 1.0.

    Returns
    -------
    cv.Variable
        The solution variable from the QP optimization.
    """
    num_steps = step_end - step_start_opt + 1
    area_sub = area_original[step_start_opt : step_end + 1]
    profile_sub = profile_matrix[:, step_start_opt : step_end + 1]

    delta_a_0 = area_sub - a_ref_input

    # Transpose profile matrix
    B = profile_sub.T
    Q = np.identity(num_steps)

    W = B.T @ Q @ B
    c = 2 * (delta_a_0.T @ Q @ B)

    # Formulate the problem to be optimized
    d_x = cv.Variable(B.shape[1])
    constraints = [d_x >= lower_bounds, d_x <= upper_bounds]
    opt_prob = cv.Problem(cv.Minimize(cv.quad_form(d_x, W) + c @ d_x + 0.1 * cv.norm(d_x, 2)), constraints)

    # We suppress ECOS solver warnings about potential inaccuracies in single iterations
    # after validating that the optimization output remains consistent and feasible.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Solution may be inaccurate.*")
        opt_prob.solve(solver=cv.ECOS, verbose=False)

    if d_x.value is None:
        raise ValueError("QP did not return a solution")

    return d_x


def perform_binary_search(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    vehicle: PlanningProblem,
    var_before: float,
    var_after: float,
    var_type: str,
    semantics: str,
    iteration_limit: int = 5,
) -> ndarray:
    """
    Performs binary search to find the highest feasible velocity (or position) between `x_before` and `x_after`
    that still yields a valid reachability graph.

    Parameters
    ----------
    scenario : Scenario
        The CommonRoad scenario being modified.

    planning_problem_set : PlanningProblemSet
        Planning problems associated with the scenario.

    vehicle : PlanningProblem
        The vehicle whose velocity or position is being adjusted. Can be used as DynamicObstacle for other vehicles in the future.

    var_before : float
        Velocity before last change. This velocity has shown to be feasible.

    var_after : float
        Velocity after last change. This velocity has shown to be infeasible.

    var_type : str
        Type of the variable - "velocity" or "position".

    semantics : str
        The semantics.

    iteration_limit : int, optional
        Maximum number of binary search steps. Default is 5.

    Returns
    -------
    np.ndarray
        1D array of drivable area values for each time step.

    """

    low = var_before
    high = var_after
    feasible_var = var_before

    # Backup current position
    x_backup, y_backup = vehicle.initial_state.position

    for iteration in range(iteration_limit):
        step_var = (low + high) / 2
        print(f"Binary search iteration {iteration+1}: Trying {var_type} = {step_var:.6f}")

        # Update ego's velocity/position
        if var_type == "velocity":
            vehicle.initial_state.velocity = step_var
        elif var_type == "x-position":
            vehicle.initial_state.position = np.array([step_var, y_backup])
        elif var_type == "y-position":
            vehicle.initial_state.position = np.array([x_backup, step_var])
        else:
            raise ValueError(f"Unsupported variable type: {var_type}")

        mod_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)

        # Reload and compute reachability
        try:
            # Check if area can be computed
            _ = reach_flow.compute_drivable_area(mod_scenario_path, semantics=semantics)

            # On success search upper half
            feasible_var = step_var
            low = step_var
        except Exception:
            # Else search lower half
            high = step_var

    print(f"Binary search complete with best feasible {var_type}: {feasible_var:.6f}")
    # Update final feasible ego's velocity/position
    if var_type == "velocity":
        vehicle.initial_state.velocity = feasible_var
    elif var_type == "x-position":
        vehicle.initial_state.position = np.array([feasible_var, y_backup])
    elif var_type == "y-position":
        vehicle.initial_state.position = np.array([x_backup, feasible_var])
    mod_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)
    area_latest = reach_flow.compute_drivable_area(mod_scenario_path, semantics=semantics)
    return area_latest


def optimize(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    iterations: int,
    a_ref_input: float = 1.0,
    semantics: str = "true",
) -> (List[float], np.ndarray):
    """
    Optimizes scenario variables (velocity or position of vehicles) to influence the drivable area.

    For each variable in `decision_variables`, it runs a loop of QP-based updates to minimize
    drivable area, reverting with binary search if updates lead to invalid configurations.

    Parameters
    ----------
    scenario : Scenario
        The CommonRoad scenario to be modified.

    planning_problem_set : PlanningProblemSet
        The set of planning problems in the scenario.

    scenario_path : str
        Path to the scenario.

    decision_variables : List[Tuple[str, str]]
        List of decision variables to optimize.
        Each tuple is (vehicle_id, variable_type), where variable_type is "velocity", "position", etc.

    iterations : int, optional
        Number of optimization iterations to run per variable. Default is 10.

    a_ref_input : float, optional
        The area reference target. Default is 1.0.

    semantics : str, optional
        The semantics. Default is "true".

    Returns
    -------
    List[float]
        List containing the final values of velocity, x-position, and y-position for the ego vehicle.

    np.ndarray
        The final drivable area.
    """

    lower_bounds = []
    upper_bounds = []
    expanded_variables = []

    # Create bounds for the QP constraints for each iteration
    # Position has smaller constraints, as it is less stable than velocity
    for vehicle_id, var_type in decision_variables:
        if var_type == "position":
            expanded_variables.append((vehicle_id, "x-position"))
            lower_bounds.append(-0.5)
            upper_bounds.append(0.5)
            expanded_variables.append((vehicle_id, "y-position"))
            lower_bounds.append(-0.5)
            upper_bounds.append(0.5)
        else:
            expanded_variables.append((vehicle_id, var_type))
            lower_bounds.append(-5.0)
            upper_bounds.append(5.0)

    lower_bounds = np.array(lower_bounds)
    upper_bounds = np.array(upper_bounds)

    # Try computing reachability with current velocity
    try:
        graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
            scenario_path, semantics=semantics
        )

    except Exception as e:
        raise Exception(f"Reachability failed: {e}")

    # Compute area
    area_latest = reach_flow.compute_drivable_area(scenario_path, semantics=semantics)
    current_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)

    # Save the best area so far
    area_best = area_latest.copy()
    best_velocity = list(planning_problem_set.planning_problem_dict.values())[0].initial_state.velocity
    best_position = list(planning_problem_set.planning_problem_dict.values())[0].initial_state.position.copy()

    for i in range(iterations):
        print("Starting iteration", i)
        # Compute profile matrix
        profile_matrix, profile_index_map = profile_matrix_computation.get_profile_matrix(
            scenario, planning_problem_set, current_scenario_path, step_start, step_end, expanded_variables, semantics
        )
        print(f"Profile: {profile_matrix}")

        # Solve QP
        d_x = optimize_iteration(
            area_latest,
            profile_matrix,
            step_end,
            lower_bounds=lower_bounds,
            upper_bounds=upper_bounds,
            a_ref_input=a_ref_input,
        )
        if d_x.value is None:
            raise ValueError("QP was not solved completely")

        # Update variables
        for idx, (vehicle_id, variable_type) in enumerate(expanded_variables):
            if vehicle_id == "ego":
                target_vehicle = list(planning_problem_set.planning_problem_dict.values())[0]
            else:
                raise ValueError(f"Program supports only ego vehicle currently")

            # Get the index of the variable in d_x corresponding to the current loop variable
            var_index = profile_index_map.get((vehicle_id, variable_type))

            if var_index is None:
                print(f"Warning: No profile found for ({vehicle_id}, {variable_type})")
                continue

            # Scale update to ensure conservative changes for feasibility
            delta = (float(d_x.value[var_index])) * 0.5
            last_change = delta

            # Apply the update
            if "position" in variable_type:
                direction = "x" if variable_type.startswith("x") else "y"
                file_modification.apply_update(target_vehicle, "position", delta, direction=direction)
            else:
                file_modification.apply_update(target_vehicle, variable_type, delta)
            print(f"Updated {variable_type} of {vehicle_id} by {delta:.4f}")

            # Save scenario
            current_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)

            # Check if the variable is feasible
            try:
                area_latest = reach_flow.compute_drivable_area(current_scenario_path, semantics=semantics)

            except Exception as e:
                print(f"Reachability failed: {e}. Performing binary search.")

                if variable_type == "velocity":
                    var_before = target_vehicle.initial_state.velocity - last_change
                    var_after = target_vehicle.initial_state.velocity
                else:
                    index = 0 if direction == "x" else 1
                    var_before = target_vehicle.initial_state.position[index] - last_change
                    var_after = target_vehicle.initial_state.position[index]

                # Run binary search between previous valid and current, invalid variable
                area_latest = perform_binary_search(
                    scenario,
                    planning_problem_set,
                    target_vehicle,
                    var_before,
                    var_after,
                    var_type=variable_type,
                    semantics=semantics,
                )

        if sum((area_latest - a_ref_input) ** 2) < sum((area_best - a_ref_input) ** 2):
            area_best = area_latest.copy()
            best_velocity = target_vehicle.initial_state.velocity
            best_position = target_vehicle.initial_state.position.copy()

    # Save the final modified scenario and return the best solution
    target_vehicle.initial_state.velocity = best_velocity
    target_vehicle.initial_state.position = best_position
    last_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)
    area_end = reach_flow.compute_drivable_area(last_scenario_path, semantics=semantics)

    velocity = target_vehicle.initial_state.velocity
    x_pos = target_vehicle.initial_state.position[0]
    y_pos = target_vehicle.initial_state.position[1]

    return [velocity, x_pos, y_pos], area_end
