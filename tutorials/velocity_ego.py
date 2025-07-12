import sys
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader

import reach_flow
from optimization import optimize


def run_full_optimization_pipeline(
    scenario_path: str, decision_variables: list, iterations: int = 5, a_ref_input: float = 1.0, semantics: str = "true"
) -> None:
    # Load scenario and compute the reachability graph and drivable area
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(scenario_path, semantics)
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)
    area_original = reach_flow.compute_drivable_area(scenario_path, semantics)

    # Run gradient-based optimization
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

    # Create reach graph for modified scenario
    mod_scenario_path = PROJECT_ROOT / "scenarios" / "modified_scenario.xml"
    scenario, planning_problem_set = CommonRoadFileReader(mod_scenario_path).open()
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        str(mod_scenario_path), semantics
    )

    # Draw reachable sets for modified scenario
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    # Plot areas
    reach_flow.plot(area_original, area_modified)


# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
# Add src and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))

# Choose a scenario as an input
# scenario_path = PROJECT_ROOT / "scenarios" / "USA_US101-8_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Guetersloh-65_2_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"
scenario_path = PROJECT_ROOT / "scenarios" / "ZAM_two_lanes_solid.xml"

run_full_optimization_pipeline(str(scenario_path), [("ego", "velocity")])
