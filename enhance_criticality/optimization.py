import os

import cvxpy as cv
import numpy as np
import reachability
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface


# Minimize changes in velocity by trying to achive reference area
def optimize_iteration(area_original, profile_matrix, steps: int, a_ref_input):
    delta_a_0 = area_original
    a_ref = np.empty([len(area_original)])
    for i in range(0, len(a_ref)):
        a_ref[i] = a_ref_input
        delta_a_0[i] = delta_a_0[i] - a_ref[i]

    B = np.transpose(profile_matrix)
    Q = np.identity(steps + 1)

    W = np.dot(np.transpose(B), Q)
    W = np.dot(W, B)

    delta_a_0_transposed = np.transpose(delta_a_0)

    q_b = np.dot(Q, B)
    q_transposed_b = np.dot(np.transpose(Q), B)
    c = q_b + q_transposed_b
    c = np.dot(delta_a_0_transposed, c)

    d_x = cv.Variable(len(profile_matrix))
    constraints = []
    # print('d_x:', d_x.size, 'W:', W.shape)
    # print('c:', c.shape)

    # TODO is it c_transposed?

    opt_prob = cv.Problem(cv.Minimize(cv.quad_form(d_x, W) + c * d_x), constraints)
    opt_prob.solve(solver=cv.ECOS, verbose=False)

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

        temp_file = os.path.join("scenarios", "modified_scenario.xml")

        writer = CommonRoadFileWriter(scenario, planning_problem_set)
        writer.write_to_file(temp_file, overwrite_existing_file=OverwriteExistingFile.ALWAYS)

        # Reload and compute reachability
        try:
            new_config = ConfigurationBuilder(path_root="../..").build_configuration(
                "modified_scenario"
            )
            new_config.update()
            reach_interface_new = ReachableSetInterface(new_config)
            reach_interface_new.compute_reachable_sets()
            # Check if area can be computed
            _ = reachability.compute_full_drivable_area(reach_interface_new)

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


def optimize_velocity(
    scenario,
    planning_problem_set,
    scenario_name,
    vehicle,
    steps: int,
    iterations: int = 10,
    a_ref_input: float = 1.0,
):
    last_change = 0.0

    for i in range(iterations):
        print(f"Iteration {i+1}: Starting with velocity = {vehicle.initial_state.velocity}")

        # Try computing reachability with current velocity
        try:
            reach_interface = reachability.compute_reachable_sets(scenario_name)

        except Exception as e:
            print(f"Reachability failed: {e}")
            # Run binary search between previous valid and current velocity
            return perform_binary_search_velocity(
                scenario,
                planning_problem_set,
                vehicle,
                x_before=vehicle.initial_state.velocity - last_change,
                x_after=vehicle.initial_state.velocity,
            )

        # Compute area and profile matrix
        area_original = reachability.compute_full_drivable_area(reach_interface)
        profile_matrix = reachability.get_profile_matrix(
            scenario, planning_problem_set[0], reach_interface
        )

        # Solve QP
        try:
            d_x = optimize_iteration(area_original, profile_matrix, steps, a_ref_input=a_ref_input)
            if d_x.value is None:
                raise ValueError("QP was not solved completely")
        except Exception as e:
            print(f"QP optimization failed: {e}")
            return vehicle.initial_state.velocity

        # Update velocity
        # TODO for now it updates the velocity for the last vehicle, which is the ego vehicle
        # TODO check if it should be * 9
        velocity_update = float(d_x.value[-1]) * 9
        last_change = velocity_update
        vehicle.initial_state.velocity += velocity_update

        # Update scenario
        modified_scenario_name = reachability.save_modified_scenario()
        _ = reachability.compute_reachable_sets(modified_scenario_name)

        print(
            f"Optimization solution: Δv = {velocity_update:.4f}, new velocity = {vehicle.initial_state.velocity:.4f}"
        )

    return vehicle.initial_state.velocity
