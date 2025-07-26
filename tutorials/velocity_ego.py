import sys
import time
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
# Add src and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))
import optimization
import reach_flow


def run_full_optimization_pipeline(
    scenario_path: str,
    decision_variables: list,
    iterations: int = 5,
    a_ref_input: float = 1.0,
    semantics: str = "true",
) -> None:
    # Load scenario and compute the reachability graph and drivable area
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(scenario_path, semantics)
    print(f"initial velocity: {planning_problem.initial_state.velocity}")
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)
    area_original = reach_flow.compute_drivable_area(scenario_path, semantics)

    # Run gradient-based optimization
    start = time.time()
    final_params, area_modified = optimization.optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
        semantics=semantics,
    )
    end = time.time()
    print("final_params:", final_params)

    # Create reach graph for modified scenario
    name = Path(scenario_path).stem
    mod_scenario_path = PROJECT_ROOT / "scenarios" / f"{name}_updated_gradient.xml"
    scenario, planning_problem_set = CommonRoadFileReader(mod_scenario_path).open()
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        str(mod_scenario_path), semantics
    )

    # Draw reachable sets for modified scenario
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    # Plot areas
    reach_flow.plot(area_original, area_modified)
    print(f"first: {sum((area_original - a_ref_input) ** 2)}")
    print(f"second: {sum((area_modified - a_ref_input) ** 2)}")
    print(f"Gradient optimization time: {end - start:.2f} seconds")


# Choose a scenario as an input
# scenario_path = PROJECT_ROOT / "scenarios" / "USA_US101-8_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Guetersloh-65_2_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"
scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-26_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "ZAM_two_lanes_solid.xml"

run_full_optimization_pipeline(str(scenario_path), [("ego", "velocity")])
