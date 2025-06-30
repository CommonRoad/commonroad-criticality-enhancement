import os
import sys
from pathlib import Path

import reach_flow
from commonroad.common.file_reader import CommonRoadFileReader
from optimization import optimize


def run_full_optimization_pipeline(
    scenario_path: str, decision_variables: list, iterations: int = 5, a_ref_input: float = 1.0
) -> None:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(scenario_path)
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    area_original = reach_flow.compute_drivable_area(scenario_path)

    final_velocity, area_modified = optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
    )
    print("final_velocity:", final_velocity)

    # scenario_file = Path(__file__).parent.joinpath(f"./../{scenario_path}")
    # scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
    #
    # graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
    #     "scenarios/modified_scenario.xml"
    # )
    # reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    reach_flow.plot(area_original, area_modified)


# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
# Add core and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "core"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))
scenario_path = PROJECT_ROOT / "scenarios" / "USA_US101-8_1_T-1.xml"
# run_full_optimization_pipeline(str(scenario_path), [("ego", "position")])
run_full_optimization_pipeline(str(scenario_path), [("ego", "velocity")])
