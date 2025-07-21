import sys
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader

import reach_flow
from optimization import optimize


def run_full_optimization_pipeline(
    scenario_path: str,
    decision_variables: list,
    iterations: int = 10,
    a_ref_input: float = 1.0,
    semantics: str = "true",
) -> None:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # Print vehicle IDs for easier defining semantics (e.g. "Behind_V311")
    vehicle_ids = [obstacle.obstacle_id for obstacle in scenario.dynamic_obstacles]
    print("Vehicle IDs:", vehicle_ids)

    # compute the reachability graph and drivable area
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(scenario_path, semantics)
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    area_original = reach_flow.compute_drivable_area(scenario_path, semantics)

    # Run gradient-based optimization
    final_params, area_modified = optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
        semantics=semantics,
    )
    print("final_params:", final_params)

    # Create reach graph for modified scenario
    mod_scenario_path = PROJECT_ROOT / "scenarios" / "updated_scenario_gradient.xml"
    scenario, planning_problem_set = CommonRoadFileReader(mod_scenario_path).open()

    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        str(mod_scenario_path), semantics
    )
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)
    reach_flow.plot(area_original, area_modified)

    print(f"original area: {sum(area_original)}")
    print(f"modified area: {sum(area_modified)}")

    return sum(area_modified)


# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
# Add src and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))

scenario_path = PROJECT_ROOT / "scenarios" / "BEL_Aarschot-6_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "USA_US101-8_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Moelln-7_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "ZAM_two_lanes_solid.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "BEL_Putte-3_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "C-DEU_B471-1_3_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_IV21-2_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "USA_US101-11_4_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "ITA_Empoli-2_5_T-1.xml"

run_full_optimization_pipeline(str(scenario_path), [("ego", "position"), ("ego", "velocity")])
