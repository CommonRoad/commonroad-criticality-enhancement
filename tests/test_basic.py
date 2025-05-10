import os
import sys

# Add the parent directory (my_project) to the system path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "enhance_criticality"))
)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenario")))
import pathlib

from commonroad.common.file_reader import CommonRoadFileReader

# from optimization import optimize


def test_run_without_err(scenario_path: str, decision_variables: list[tuple[str, str]]):
    scenario_file = pathlib.Path(__file__).parent.joinpath(f"./../{scenario_path}")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    # optimize(
    #     scenario,
    #     planning_problem_set,
    #     scenario_path,
    #     vehicle=planning_problem_set,
    #     decision_variables=decision_variables,
    # )


test_run_without_err("scenarios/ZAM_Merge-1_1_T-1.xml", [("ego", "velocity")])
