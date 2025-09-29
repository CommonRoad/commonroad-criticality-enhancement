import sys
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_criticality_enhancement import optimization, reach_flow

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.append(str(PROJECT_ROOT / "scenarios"))


def run_full_optimization_pipeline(
    scenario_path: str,
    decision_variables: list,
    iterations: int = 5,
    a_ref_input: float = 1.0,
    semantics: str = "true",
) -> int:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # Print vehicle IDs for easier defining semantics (e.g. "Behind_V311")
    vehicle_ids = [obstacle.obstacle_id for obstacle in scenario.dynamic_obstacles]
    print("Vehicle IDs:", vehicle_ids)

    # compute the reachability graph and drivable area
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        scenario, planning_problem_set, semantics
    )

    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    area_original = reach_flow.compute_drivable_area(scenario, planning_problem_set, semantics)

    # Run gradient-based optimization
    final_params, area_modified = optimization.optimize(
        scenario,
        planning_problem_set,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
        semantics=semantics,
    )
    print("final_params:", final_params)

    # Create reach graph for modified scenario
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        scenario, planning_problem_set, semantics
    )
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)
    reach_flow.plot(area_original, area_modified)

    print(f"original objective: {(sum(area_original) - a_ref_input) ** 2}")
    print(f"modified objective: {(sum(area_modified) - a_ref_input) ** 2}")

    return sum((area_modified - a_ref_input) ** 2)


scenario_path = PROJECT_ROOT / "scenarios" / "USA_US101-11_4_T-1.xml"

run_full_optimization_pipeline(str(scenario_path), [("ego", "velocity"), ("ego", "position")])
