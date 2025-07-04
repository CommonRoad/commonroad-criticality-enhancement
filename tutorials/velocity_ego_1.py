import os
import sys
from pathlib import Path

import reach_flow
from commonroad.common.file_reader import CommonRoadFileReader
from optimization import optimize


def run_full_optimization_pipeline(
    scenario_path: str, decision_variables: list, iterations: int = 5, a_ref_input: float = 1.0, semantics: str = "true"
) -> None:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(scenario_path, semantics)
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    # To make further adjustments to the way the reachable area is computed,
    # change the point mass parameters in the create_reach_graph() method.
    #   e.g. by constraining the lateral velocity with > 0, the vehicle will only move forward
    #       and there will be no area behind the vehicle
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

    mod_scenario_path = PROJECT_ROOT / "scenarios" / "modified_scenario.xml"
    scenario, planning_problem_set = CommonRoadFileReader(mod_scenario_path).open()

    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        str(mod_scenario_path), semantics
    )
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    reach_flow.plot(area_original, area_modified)


# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
# Add core and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "core"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))
# scenario_path = PROJECT_ROOT / "scenarios" / "USA_US101-8_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Guetersloh-65_2_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"
scenario_path = PROJECT_ROOT / "scenarios" / "ZAM_two_lanes_solid.xml"
# run_full_optimization_pipeline(str(scenario_path), [("ego", "position")])
run_full_optimization_pipeline(str(scenario_path), [("ego", "velocity")])
