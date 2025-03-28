import commonroad_reach.utility.logger as util_logger
import numpy as np
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual


# Loads a scenario from an xml file. params: scenario (str) - name of the scenario
# Computes reachable set of ego vehicle and outputs it as a gif
def load_scenario_and_compute_reachability(scenario):
    # Build configuration
    config = ConfigurationBuilder(path_root="../..").build_configuration(scenario)
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
    derivative_ego_velocity = differentiate_reachable_set_wrt_velocity(
        reach_interface, scenario.ego_vehicle
    )
    print(derivative_ego_velocity)

    # Plot computation results
    util_visual.plot_scenario_with_reachable_sets(reach_interface, figsize=(7, 7))

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


# Computes the derivative of the reachable set area with respect to ego vehicle velocity using the h-method.
def differentiate_reachable_set_wrt_velocity(reach_interface, vehicle, h=1e-3):
    original_velocity = vehicle.velocity

    # Compute reachable area fororiginal velocity
    reach_interface.compute_reachable_sets()
    area_original = compute_full_drivable_area(reach_interface)

    # Slightly increase velocity
    vehicle.velocity += h
    reach_interface.compute_reachable_sets()
    area_changed_velocity = compute_full_drivable_area(reach_interface)

    # Derivative using h method
    derivative = (area_changed_velocity - area_original) / h

    vehicle.velocity = original_velocity

    return derivative


# TODO adjust the h_y if necessary: one function for x and y or two separate


def differentiate_reachable_set_wrt_position(reach_interface, vehicle, h_x=0.001):
    original_position = vehicle.x

    # Compute reachable area fororiginal velocity
    reach_interface.compute_reachable_sets()
    area_original = compute_full_drivable_area(reach_interface)

    # Slightly increase velocity
    vehicle.x += h_x
    reach_interface.compute_reachable_sets()
    area_changed_position = compute_full_drivable_area(reach_interface)

    # Derivative using h method
    derivative = (area_changed_position - area_original) / h_x

    vehicle.x = original_position

    return derivative


def get_profile_matrix(scenario):
    result = []

    # Compute reachability profiles for every vehicle (does not include ego vehicle)
    for obs in scenario.dynamic_obstacles:
        profile_pos = differentiate_reachable_set_wrt_other_vehicle_position(scenario, obs)
        profile_vel = differentiate_reachable_set_wrt_other_vehicle_velocity(scenario, obs)

        result.append(profile_pos)
        result.append(profile_vel)

    # Compute reachability profiles for the ego vehicle TODO
    ego_pos = differentiate_reachable_set_wrt_position(scenario, scenario.ego_vehicle)
    ego_vel = differentiate_reachable_set_wrt_velocity(scenario, scenario.ego_vehicle)

    result.append(ego_pos)
    result.append(ego_vel)

    # Transform List to matrix
    profile_matrix = np.vstack(result)
    return profile_matrix


scenario = "ZAM_Tjunction-1_307_T-1"
reachable_sets = load_scenario_and_compute_reachability(scenario)
