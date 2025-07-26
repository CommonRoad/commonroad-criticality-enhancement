import sys
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from numpy import ndarray

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
# Add src and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))
import file_modification
import reach_flow
from optimization import optimize


def run_full_optimization_pipeline(
    scenario_path: str,
    decision_variables: list,
    iterations: int = 40,
    a_ref_input: float = 1.0,
    semantics: str = "true",
) -> ndarray:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # Print vehicle IDs for easier defining semantics (e.g. "Behind_V311")
    # vehicle_ids = [obstacle.obstacle_id for obstacle in scenario.dynamic_obstacles]
    # print("Vehicle IDs:", vehicle_ids)

    # compute the reachability graph and drivable area
    # graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(scenario_path, semantics)

    # saved_file = file_modification.save_modified_scenario(scenario, planning_problem_set)
    # scenario, planning_problem_set = CommonRoadFileReader(saved_file).open()

    # reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    area_original = reach_flow.compute_drivable_area(scenario_path, semantics)
    # area_new = reach_flow.compute_drivable_area(saved_file, semantics)

    # print(f"{(sum(area_original), sum(area_new))}")

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

    # # Create reach graph for modified scenario
    # name = Path(scenario_path).stem
    # mod_scenario_path = PROJECT_ROOT / "scenarios" / f"{name}_updated_gradient.xml"
    # scenario, planning_problem_set = CommonRoadFileReader(mod_scenario_path).open()
    #
    # graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
    #     str(mod_scenario_path), semantics
    # )
    # reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)
    # reach_flow.plot(area_original, area_modified)

    print(f"original area: {sum(area_original)}")
    print(f"modified area: {sum(area_modified)}")
    print(f"first: {sum((area_original - a_ref_input) ** 2)}")
    print(f"second: {sum((area_modified - a_ref_input) ** 2)}")

    return sum((area_modified - a_ref_input) ** 2)


# scenario_path = PROJECT_ROOT / "scenarios" / "BEL_Aarschot-6_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "USA_US101-8_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Moelln-7_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "ZAM_two_lanes_solid.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "BEL_Putte-3_1_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "C-DEU_B471-1_3_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "DEU_IV21-2_1_T-1.xml"
scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-26_1_T-1.xml"

run_full_optimization_pipeline(str(scenario_path), [("ego", "position"), ("ego", "velocity")])
