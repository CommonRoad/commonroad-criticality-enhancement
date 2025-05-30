import os
import sys
from pathlib import Path

import reach_flow

# Add the parent directory (my_project) to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "core")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenario")))

from commonroad.common.file_reader import CommonRoadFileReader
from optimization import optimize


def run_full_optimization_pipeline(
    scenario_path: str, decision_variables: list, iterations: int = 9, a_ref_input: float = 1.0
) -> None:
    scenario_file = Path(__file__).parent.joinpath(f"./../{scenario_path}")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        scenario_path
    )
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    area_original = reach_flow.compute_drivable_area(scenario_path)

    final_velocity = optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
    )
    print("final_velocity:", final_velocity)

    scenario_file = Path(__file__).parent.joinpath(f"./../{scenario_path}")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        "scenarios/modified_scenario.xml"
    )
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    area_modified = reach_flow.compute_drivable_area("scenarios/modified_scenario.xml")
    reach_flow.plot(area_original, area_modified)


run_full_optimization_pipeline("scenarios/DEU_Reutlingen-5_1_T-1.xml", [("ego", "velocity")])
