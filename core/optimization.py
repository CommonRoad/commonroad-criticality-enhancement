from typing import List, Tuple

import cvxpy as cv
import file_modification
import numpy as np
import profile_matrix_computation
import reach_flow
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario


def optimize_iteration(
    area_original: np.ndarray,
    profile_matrix: np.ndarray,
    step_end: int,
    constraints_const: int,
    step_start_opt: int = 6,
    a_ref_input: float = 1.0,
) -> cv.Variable:
    """
    Solves a quadratic program (QP) to compute optimal adjustments to decision variables.

    Parameters:
    - area_original (np.ndarray): The original drivable area over time.
    - profile_matrix (np.ndarray): Sensitivity matrix showing how each decision variable affects the area.
    - step_end (int): Last time step.
    - constraints_const (int): Integer to regulate the step in an optimization iteration.
    - step_start (int): Time step to start the optimization (default: 4).
    - a_ref_input (float, optional): Scalar to modify the area reference target. Default is 1.0.

    Returns:
    - cv.Variable: The solution variable from the QP optimization.
    """

    num_steps = step_end - step_start_opt + 1
    area_sub = area_original[step_start_opt : step_end + 1]
    profile_sub = profile_matrix[:, step_start_opt : step_end + 1]

    delta_a_0 = area_sub - a_ref_input

    delta_a_0 = np.maximum(delta_a_0, 1e-2)

    B = profile_sub.T
    Q = np.identity(num_steps)

    W = B.T @ Q @ B
    c = 2 * (delta_a_0.T @ Q @ B)

    d_x = cv.Variable(B.shape[1])
    constraints = [d_x >= -constraints_const, d_x <= constraints_const]
    opt_prob = cv.Problem(cv.Minimize(cv.quad_form(d_x, W) + c @ d_x + 0.1 * cv.norm(d_x, 2)), constraints)
    opt_prob.solve(solver=cv.ECOS, verbose=False)

    if d_x.value is None:
        raise ValueError("QP did not return a solution")

    return d_x


def perform_binary_search(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    vehicle: DynamicObstacle,
    var_before: float,
    var_after: float,
    var_type: str,
    semantics: str,
    iteration_limit: int = 10,
) -> None:
    """
    Performs binary search to find the highest feasible velocity (or position) between `x_before` and `x_after`
    that still yields a valid reachability graph.

    Parameters:
    - scenario (Scenario): The CommonRoad scenario being modified.
    - planning_problem_set (PlanningProblemSet): Planning problems associated with the scenario.
    - vehicle (DynamicObstacle): The vehicle whose velocity is being adjusted.
    - var_before (float): Velocity before last change.
    - var_after (float): Velocity after last change.
    - var_type (str): Type of the variable - "velocity" or "position".
    - semantics (str): The semantics.
    - iteration_limit (int, optional): Maximum number of binary search steps. Default is 10.
    """

    low = var_before
    high = var_after
    feasible_var = var_before
    # Backup current state
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
    _ = reach_flow.compute_drivable_area(mod_scenario_path, semantics=semantics)


def apply_update(target_vehicle: DynamicObstacle, variable_type: str, delta: float, direction: str = "x") -> None:
    """
    Applies a delta update to a specified decision variable of a vehicle.

    Parameters:
    - target_vehicle (DynamicObstacle): The vehicle to be modified.
    - variable_type (str): The variable to update ("velocity" or "position").
    - delta (float): The amount to adjust the variable by.
    - direction (str, optional): The direction of the variable. Default is "x".

    Raises:
    - ValueError: If the resulting velocity is non-positive or an unsupported variable type is provided.
    """
    if variable_type == "velocity":
        target_vehicle.initial_state.velocity += delta
        if target_vehicle.initial_state.velocity <= 0:
            raise ValueError("Velocity cannot be negative")
    elif variable_type == "position":
        x, y = target_vehicle.initial_state.position
        if direction == "x":
            new_pos = np.array([x + delta, y])
        else:
            new_pos = np.array([x, y + delta])
        target_vehicle.initial_state.position = new_pos
        # update_pos_trajectory(target_vehicle, delta)
    else:
        raise ValueError(f"Unsupported variable type: {variable_type}")


def optimize(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    iterations: int,
    a_ref_input: float = 1.0,
    semantics: str = "true",
) -> (float, np.ndarray):
    """
    Optimizes scenario variables (velocity or position of vehicles) to influence the drivable area.

    For each variable in `decision_variables`, it runs a loop of QP-based updates to maximize
    reachability, reverting with binary search if updates lead to invalid configurations.

    Parameters:
    - scenario (Scenario): The CommonRoad scenario being modified.
    - planning_problem_set (PlanningProblemSet): The set of planning problems in the scenario.
    - scenario_path (str): Path to the scenario.
    - decision_variables (List[Tuple[str, str]]): List of decision variables to optimize.
        Each tuple is (vehicle_id, variable_type), where variable_type is "velocity", "position" etc.
    - iterations (int, optional): Number of optimization iterations to run per variable. Default is 10.
    - a_ref_input (float, optional): Scalar to modify the area reference target. Default is 1.0.
    - semantics (str, optional): The semantics. Default is "true".

    Returns:
    - float: The highest feasible velocity found.
    - np.ndarray: The final drivable area
    """
    constraints_const = 5
    expanded_variables = []
    for vehicle_id, var_type in decision_variables:
        if var_type == "position":
            expanded_variables.append((vehicle_id, "x-position"))
            expanded_variables.append((vehicle_id, "y-position"))
            constraints_const = 2
        else:
            expanded_variables.append((vehicle_id, var_type))

    # Try computing reachability with current velocity
    try:
        graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
            scenario_path, semantics=semantics
        )

    except Exception as e:
        raise Exception(f"Reachability failed: {e}")

    # Compute area and profile matrix
    area_latest = reach_flow.compute_drivable_area(scenario_path, semantics=semantics)
    current_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)

    for i in range(iterations):
        print("Starting iteration", i)
        profile_matrix, profile_index_map = profile_matrix_computation.get_profile_matrix(
            scenario, planning_problem_set, current_scenario_path, step_start, step_end, expanded_variables, semantics
        )
        print(f"Profile: {profile_matrix}")

        # Solve QP
        try:
            d_x = optimize_iteration(area_latest, profile_matrix, step_end, constraints_const, a_ref_input=a_ref_input)
            if d_x.value is None:
                raise ValueError("QP was not solved completely")
        except Exception as e:
            print(f"Warning: QP optimization failed: {e}")

        # Update variable
        for idx, (vehicle_id, variable_type) in enumerate(expanded_variables):
            if vehicle_id == "ego":
                target_vehicle = list(planning_problem_set.planning_problem_dict.values())[0]
            else:
                raise ValueError(f"Program supports only ego vehicle currently")

            if target_vehicle is None:
                print(f"Warning: Vehicle with ID {vehicle_id} not found.")
                continue

            # Get the index of the variable in d_x corresponding to the current loop variable
            var_index = profile_index_map.get((vehicle_id, variable_type))

            if var_index is None:
                print(f"Warning: No profile found for ({vehicle_id}, {variable_type})")
                continue

            # Scale update to ensure conservative changes for feasibility
            delta = (float(d_x.value[var_index])) * 0.5
            last_change = delta
            if "position" in variable_type:
                direction = "x" if variable_type.startswith("x") else "y"
                apply_update(target_vehicle, "position", delta, direction=direction)
            else:
                apply_update(target_vehicle, variable_type, delta)
            print(f"Updated {variable_type} of {vehicle_id} by {delta:.4f}")

            # Update scenario
            current_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)

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
                # Run binary search between previous valid and current velocity
                perform_binary_search(
                    scenario,
                    planning_problem_set,
                    target_vehicle,
                    var_before,
                    var_after,
                    var_type=variable_type,
                    semantics=semantics,
                )

    last_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)
    area_end = reach_flow.compute_drivable_area(last_scenario_path, semantics=semantics)

    if expanded_variables[-1][1] == "velocity":
        return target_vehicle.initial_state.velocity, area_end
    elif expanded_variables[-1][1].startswith("x"):
        return target_vehicle.initial_state.position[0], area_end
    else:
        return target_vehicle.initial_state.position[1], area_end
