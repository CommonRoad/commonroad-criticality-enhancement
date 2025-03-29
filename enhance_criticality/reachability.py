import os
import pathlib

import commonroad_reach.utility.logger as util_logger
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface


# Loads a scenario from an xml file. params: scenario (str) - name of the scenario
# Computes reachable set of ego vehicle and outputs it as a gif
def load_scenario_and_compute_reachability(scenario_name):
    # Build configuration
    config = ConfigurationBuilder(path_root="../..").build_configuration(scenario_name)
    config.update()

    # Initialize logger
    logger = util_logger.initialize_logger(config)
    config.print_configuration_summary()

    # Compute reachable sets
    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets()

    # Compute the drivable area at each step
    drivable_area = compute_full_drivable_area(reach_interface)
    print("The drivable area at the start is:", drivable_area[0])

    # Plot computation results
    # util_visual.plot_scenario_with_reachable_sets(reach_interface, figsize=(7, 7))

    return reach_interface


def compute_drivable_slice_area(drivable_slice):
    area = 0

    for region in drivable_slice:
        width = region.p_lon_max - region.p_lon_min
        height = region.p_lat_max - region.p_lat_min
        area += width * height

    return area


# area-profile = development of the drivable area over discrete times
def compute_full_drivable_area(reach_interface):
    drivable_area = np.array(
        [-1.0 for i in range(reach_interface.step_start, reach_interface.step_end + 1)]
    )

    for time_step in range(reach_interface.step_start, reach_interface.step_end + 1):
        drivable_slice = reach_interface.drivable_area_at_step(time_step)
        area = compute_drivable_slice_area(drivable_slice)
        drivable_area[time_step] = area

    return drivable_area


# Computes the derivative of the reachable set area with respect to vehicle velocity using the h-method
# TODO adjust new positions
def differentiate_reachable_set_wrt_velocity(reach_interface, scenario_name, vehicle):
    original_velocity = vehicle.initial_state.velocity
    h = original_velocity / 10

    derivative = np.array(
        [-1.0 for i in range(reach_interface.step_start, reach_interface.step_end + 1)]
    )

    # Compute reachable area fororiginal velocity
    reach_interface.compute_reachable_sets()
    area_original = compute_full_drivable_area(reach_interface)

    # Slightly increase velocity
    vehicle.initial_state.velocity += h

    print("original_velocity: ", original_velocity)
    print("modified velocity: ", vehicle.initial_state.velocity)

    # Save the modified scenario
    temp_filename = os.path.join("scenarios", "modified_scenario.xml")
    writer = CommonRoadFileWriter(scenario, planning_problem_set)
    writer.write_to_file(temp_filename, overwrite_existing_file=OverwriteExistingFile.ALWAYS)

    # Reload the modified scenario
    new_config = ConfigurationBuilder(path_root="../..").build_configuration("modified_scenario")
    new_config.update()
    reach_interface_new = ReachableSetInterface(new_config)
    reach_interface_new.compute_reachable_sets()

    area_changed_velocity = compute_full_drivable_area(reach_interface_new)

    print("original area: ", area_original)
    print("modified area: ", area_changed_velocity)

    # Derivative using h method
    for time_step in range(reach_interface.step_start, reach_interface.step_end + 1):
        derivative[time_step] = (area_changed_velocity[time_step] - area_original[time_step]) / h

    vehicle.initial_state.velocity = original_velocity

    return derivative


# TODO adjust the h_y if necessary: one function for x and y or two separate, ego vehicle position?


def differentiate_reachable_set_wrt_position(reach_interface, vehicle, scenario_max_time):
    original_position = vehicle.initial_state.position

    delta_x = int(np.ceil(scenario_max_time / 10))
    delta_x = max(delta_x, 1)

    derivative = np.array(
        [-1.0 for i in range(reach_interface.step_start, reach_interface.step_end + 1)]
    )

    # Compute reachable area fororiginal velocity
    reach_interface.compute_reachable_sets()
    area_original = compute_full_drivable_area(reach_interface)

    # Slightly increase velocity
    vehicle.initial_state.position = (
        vehicle.initial_state.position[0] + delta_x,
        vehicle.initial_state.position[1],
    )
    print("original position: ", original_position)
    print("modified position: ", vehicle.initial_state.position)

    # Save the modified scenario
    temp_filename = os.path.join("scenarios", "modified_scenario.xml")
    writer = CommonRoadFileWriter(scenario, planning_problem_set)
    writer.write_to_file(temp_filename, overwrite_existing_file=OverwriteExistingFile.ALWAYS)

    # Reload the modified scenario
    new_config = ConfigurationBuilder(path_root="../..").build_configuration("modified_scenario")
    new_config.update()
    reach_interface_new = ReachableSetInterface(new_config)
    reach_interface_new.compute_reachable_sets()
    area_changed_position = compute_full_drivable_area(reach_interface_new)

    print("original area: ", area_original)
    print("modified area: ", area_changed_position)

    # Derivative using h method
    for time_step in range(reach_interface.step_start, reach_interface.step_end + 1):
        derivative[time_step] = (
            area_changed_position[time_step] - area_original[time_step]
        ) / delta_x

    vehicle.initial_state.position = original_position

    return derivative


def get_profile_matrix(
    scenario, planning_problem, reach_interface, scenario_name, scenario_max_time
):
    result = []

    # Compute reachability profiles for every vehicle (does not include ego vehicle)
    for obs in scenario.dynamic_obstacles:
        profile_pos = differentiate_reachable_set_wrt_position(
            reach_interface, obs, scenario_max_time
        )
        profile_vel = differentiate_reachable_set_wrt_velocity(reach_interface, scenario_name, obs)

        result.append(profile_pos)
        result.append(profile_vel)

    # Compute reachability profile for the ego vehicle
    ego_vel = differentiate_reachable_set_wrt_velocity(
        reach_interface, scenario_name, planning_problem
    )

    result.append(ego_vel)

    # Transform List to matrix
    profile_matrix = np.vstack(result)
    return profile_matrix


scenario_name = "ZAM_Tjunction-1_307_T-1"
reach_interface = load_scenario_and_compute_reachability(scenario_name)

scenario_file = pathlib.Path(__file__).parent.joinpath("./../scenarios/ZAM_Tjunction-1_307_T-1.xml")
scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

# Get the first planning problem (this is the ego vehicle's problem)
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

# Print the initial state of the ego vehicle
print(planning_problem.initial_state)
profile_matrix = get_profile_matrix(scenario, planning_problem, reach_interface, scenario_name, 5)
