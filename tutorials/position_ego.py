import sys
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader

import reach_flow
from optimization import optimize


def run_full_optimization_pipeline(
    scenario_path: str,
    decision_variables: list,
    iterations: int = 20,
    a_ref_input: float = 1.0,
    semantics: str = "true",
) -> None:
    # Load scenario and compute the reachability graph and drivable area
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # Print vehicle IDs for easier defining semantics (e.g. "Behind_V8")
    vehicle_ids = [obstacle.obstacle_id for obstacle in scenario.dynamic_obstacles]
    print("Vehicle IDs:", vehicle_ids)

    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(scenario_path, semantics)
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)
    area_original = reach_flow.compute_drivable_area(scenario_path, semantics)

    # Run gradient-based optimization
    final_position, area_modified = optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
        semantics=semantics,
    )
    print("final_params:", final_position)

    # Create reach graph for modified scenario
    name = Path(scenario_path).stem
    mod_scenario_path = PROJECT_ROOT / "scenarios" / f"{name}_updated_gradient.xml"
    scenario, planning_problem_set = CommonRoadFileReader(mod_scenario_path).open()
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        str(mod_scenario_path), semantics
    )

    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    reach_flow.plot(area_original, area_modified)


# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Add src and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))

# Choose a scenario to optimize
# scenario_path_1 = PROJECT_ROOT / "scenarios" / "DEU_Flensburg-94_1_T-1.xml"
scenario_path_2 = PROJECT_ROOT / "scenarios" / "USA_US101-8_1_T-1.xml"
# run_full_optimization_pipeline(str(scenario_path_1), [("ego", "position")], semantics="Behind_V310")
# run_full_optimization_pipeline(str(scenario_path_1), [("ego", "position")], semantics="Behind_V36")

run_full_optimization_pipeline(str(scenario_path_2), [("ego", "position")])
run_full_optimization_pipeline(str(scenario_path_2), [("ego", "position")], semantics="Behind_V35")
