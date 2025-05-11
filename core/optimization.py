from __future__ import annotations

import cvxpy as cv
import file_modification
import numpy as np
import profile_matrix_computation
import reach_flow


# Minimize changes in velocity by trying to achieve reference area
def optimize_iteration(area_original, profile_matrix, steps: int, a_ref_input=1.0):
    a_ref = area_original.copy() * 0.7
    delta_a_0 = np.copy(area_original) - a_ref

    for i in range(steps):
        if delta_a_0[i] <= 0:
            raise ValueError("delta_a_0 must be positive")

    B = profile_matrix.T
    Q = np.identity(steps)

    W = B.T @ Q @ B
    c = 2 * (delta_a_0.T @ Q @ B)

    d_x = cv.Variable(B.shape[1])
    constraints = [d_x >= -5, d_x <= 5]
    # print('d_x:', d_x.size, 'W:', W.shape)
    # print('c:', c.shape)

    opt_prob = cv.Problem(cv.Minimize(cv.quad_form(d_x, W) + c @ d_x), constraints)
    opt_prob.solve(solver=cv.ECOS, verbose=True)

    return d_x


# X before and after last QP update
def perform_binary_search_velocity(
    scenario, planning_problem_set, vehicle, x_before, x_after, iteration_limit=10
):
    low = x_before
    high = x_after
    feasible_velocity = x_before

    for iteration in range(iteration_limit):
        step_velocity = (low + high) / 2
        print(f"Binary search iteration {iteration+1}: Trying velocity = {step_velocity:.6f}")

        # Update ego's velocity
        vehicle.initial_state.velocity = step_velocity

        mod_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)

        # Reload and compute reachability
        try:
            # Check if area can be computed
            _ = reach_flow.create_reach_graph(mod_scenario_path)

            # On success search upper half
            feasible_velocity = step_velocity
            low = step_velocity

        # Else search lower half
        except Exception:
            high = step_velocity

        # Stop early if step size is small
        if abs(high - low) < 1e-4:
            break

    print(f"Binary search complete with best feasible velocity: {feasible_velocity:.6f}")
    return feasible_velocity


def optimize(
    scenario,
    planning_problem_set,
    scenario_path,
    vehicle,
    decision_variables: list,
    iterations: int = 10,
    a_ref_input: float = 1.0,
):
    last_change = 0.0

    for var in decision_variables:
        for i in range(iterations):
            # Try computing reachability with current velocity
            try:
                graph, step_start, step_end = reach_flow.create_reach_graph(scenario_path)

            except Exception as e:
                raise Exception(f"Reachability failed: {e}")

            # Compute area and profile matrix
            steps = step_end - step_start + 1
            area_original = reach_flow.compute_drivable_area(scenario_path)
            profile_matrix, profile_index_map = profile_matrix_computation.get_profile_matrix(
                scenario,
                planning_problem_set,
                scenario_path,
                step_start,
                step_end,
                decision_variables,
            )

            # Solve QP
            try:
                d_x = optimize_iteration(
                    area_original, profile_matrix, steps, a_ref_input=a_ref_input
                )
                if d_x.value is None:
                    raise ValueError("QP was not solved completely")
            except Exception as e:
                print(f"Warning: QP optimization failed: {e}")

            # Update variable
            vehicle_id, variable_type = var
            if vehicle_id == "ego":
                target_vehicle = list(planning_problem_set.planning_problem_dict.values())[0]
            else:
                target_vehicle = next(
                    (veh for veh in scenario.dynamic_obstacles if veh.obstacle_id == vehicle_id),
                    None,
                )

            if target_vehicle is None:
                print(f"Warning: Vehicle with ID {vehicle_id} not found.")
                continue

            # Get the index of the variable in d_x corresponding to the current loop variable
            # Assumes order of profile_matrix rows aligns with decision_variables
            var_index = profile_index_map.get((vehicle_id, variable_type))

            if var_index is None:
                print(f"Warning: No profile found for ({vehicle_id}, {variable_type})")
                continue

            delta = float(d_x.value[var_index]) * 0.5
            last_change = delta

            # Apply the update
            if variable_type == "velocity":
                target_vehicle.initial_state.velocity += delta
                if target_vehicle.initial_state.velocity <= 0:
                    raise ValueError("Velocity cannot be negative")
                print(
                    f"Updated velocity of vehicle {vehicle_id} by {delta:.4f} to {target_vehicle.initial_state.velocity:.4f}"
                )
            elif variable_type == "position":
                target_vehicle.initial_state.position = (
                    vehicle.initial_state.position[0] + delta,
                    vehicle.initial_state.position[1],
                )
                print(
                    f"Updated position of vehicle {vehicle_id} to {target_vehicle.initial_state.position}"
                )
            else:
                print(f"Warning: Unknown variable type: {variable_type}")

            # Update scenario
            modified_scenario_path = file_modification.save_modified_scenario(
                scenario, planning_problem_set
            )

            try:
                _ = reach_flow.create_reach_graph(modified_scenario_path)

            except Exception as e:
                print(f"Reachability failed: {e}. Performing binary search.")
                # Run binary search between previous valid and current velocity
                return perform_binary_search_velocity(
                    scenario,
                    planning_problem_set,
                    target_vehicle,
                    x_before=target_vehicle.initial_state.velocity - last_change,
                    x_after=target_vehicle.initial_state.velocity,
                )
    return target_vehicle.initial_state.velocity
