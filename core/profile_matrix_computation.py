import file_modification
import numpy as np
import reach_flow


# Computes the derivative of the reachable set area with respect to vehicle velocity using the h-method
def differentiate_reachable_set_wrt_velocity(
    scenario, planning_problem_set, scenario_path, step_start, step_end, vehicle
):
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

    # print("original_velocity: ", original_velocity)
    # print("modified velocity: ", vehicle.initial_state.velocity)

    # Save the modified scenario
    mod_scenario_path = file_modification.save_modified_scenario(scenario, planning_problem_set)

    # Recompute for the modified scenario
    area_changed_velocity = reach_flow.compute_drivable_area(mod_scenario_path)

    # print("original area: ", area_original)
    # print("modified area: ", area_changed_velocity)

    # Derivative using h method
    for time_step in range(step_start, step_end + 1):
        derivative[time_step] = (area_changed_velocity[time_step] - area_original[time_step]) / h

    vehicle.initial_state.velocity = original_velocity

    return derivative


def differentiate_reachable_set_wrt_position(
    scenario,
    planning_problem_set,
    scenario_path,
    step_start,
    step_end,
    vehicle,
    scenario_max_time,
):
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
    scenario, planning_problem_set, scenario_path, step_start, step_end, decision_variables
):
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
