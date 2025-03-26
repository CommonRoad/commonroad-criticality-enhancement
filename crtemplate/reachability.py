import pathlib

import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual


# Loads a scenario from an xml file. params: scenario (str) - name of the scenario
# Computes reachable set of ego vehicle and outputs it as a gif
def load_scenario_and_compute_reachability(scenario):
    # === Build configuration
    config = ConfigurationBuilder(path_root="../..").build_configuration(scenario)
    config.update()

    # Initialize logger
    logger = util_logger.initialize_logger(config)
    config.print_configuration_summary()

    # === Compute reachable sets
    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets()

    # === Plot computation results
    util_visual.plot_scenario_with_reachable_sets(reach_interface, figsize=(7, 7))

    return reach_interface


scenario = "ZAM_Tjunction-1_307_T-1"
reachable_sets = load_scenario_and_compute_reachability(scenario)
