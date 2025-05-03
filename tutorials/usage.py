import os
import sys

# Add the parent directory (my_project) to the system path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "enhance_criticality"))
)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenario")))
import pathlib

from commonroad.common.file_reader import CommonRoadFileReader
from optimization import optimize_velocity
from reachability import load_scenario_and_compute_reachability


def run_full_optimization_pipeline(
    scenario_name: str, decision_variables: list, iterations: int = 5, a_ref_input: float = 1.0
):
    reach_interface = load_scenario_and_compute_reachability(scenario_name)

    scenario_file = pathlib.Path(__file__).parent.joinpath(f"./../scenarios/{scenario_name}.xml")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    # Print the initial state of the ego vehicle
    # print(planning_problem.initial_state)

    final_velocity = optimize_velocity(
        scenario,
        planning_problem_set,
        scenario_name,
        vehicle=planning_problem_set,
        decision_variables=decision_variables,
    )

    _ = load_scenario_and_compute_reachability("modified_scenario")


run_full_optimization_pipeline("DEU_Test-1_1_T-1", [("ego", "velocity")])
