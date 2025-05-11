import os
import sys

# Add the parent directory (my_project) to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "core")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenario")))
import pathlib

from commonroad.common.file_reader import CommonRoadFileReader
from optimization import optimize


def run_full_optimization_pipeline(
    scenario_path: str, decision_variables: list, iterations: int = 5, a_ref_input: float = 1.0
):
    # TODO visualize original scenario

    scenario_file = pathlib.Path(__file__).parent.joinpath(f"./../{scenario_path}")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    # Print the initial state of the ego vehicle
    # print(planning_problem.initial_state)

    final_velocity = optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        vehicle=planning_problem_set,
        decision_variables=decision_variables,
    )
    print("final_velocity:", final_velocity)

    # TODO visualize modified scenario


run_full_optimization_pipeline("scenarios/ZAM_Merge-1_1_T-1.xml", [("ego", "velocity")])
