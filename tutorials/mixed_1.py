import os
import pathlib
import sys
from pathlib import Path

import reach_flow
from commonroad.common.file_reader import CommonRoadFileReader
from optimization import optimize


def run_full_optimization_pipeline(
    scenario_path: str, decision_variables: list, iterations: int = 5, a_ref_input: float = 1.0, semantics: str = "true"
) -> None:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()
    vehicle_ids = [obstacle.obstacle_id for obstacle in scenario.dynamic_obstacles]
    print("Vehicle IDs:", vehicle_ids)

    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(scenario_path, semantics)
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    area_original = reach_flow.compute_drivable_area(scenario_path, semantics)

    final_velocity, area_modified = optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
        semantics=semantics,
    )
    print("final_velocity:", final_velocity)

    # scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
    #
    # graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
    #     "scenarios/modified_scenario.xml", semantics
    # )
    # reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)
    reach_flow.plot(area_original, area_modified)


# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
# Add core and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "core"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))
scenario_path = PROJECT_ROOT / "scenarios" / "BEL_Aarschot-6_1_T-1.xml"
run_full_optimization_pipeline(str(scenario_path), [("ego", "velocity"), ("ego", "position")], semantics="Behind_V311")
