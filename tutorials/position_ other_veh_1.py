import os
import sys
from pathlib import Path

import reach_flow

# Add the parent directory (my_project) to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "core")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenario")))
import pathlib

from commonroad.common.file_reader import CommonRoadFileReader
from file_modification import save_modified_scenario
from optimization import optimize


def run_full_optimization_pipeline(
    scenario_path: str, decision_variables: list, iterations: int = 9, a_ref_input: float = 1.0
) -> None:
    scenario_file = Path(__file__).parent.joinpath(f"./../{scenario_path}")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
    vehicle_ids = [obstacle.obstacle_id for obstacle in scenario.dynamic_obstacles]
    print("Vehicle IDs:", vehicle_ids)
    target_veh = next(
        (veh for veh in scenario.dynamic_obstacles if veh.obstacle_id == 30),
        None,
    )
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        scenario_path, "Behind_V311"
    )
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    area_original = reach_flow.compute_drivable_area(scenario_path)

    final_pos = optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
    )
    print("final_position:", final_pos)
    # target_vehicle.initial_state.position[0] = 441
    # save_modified_scenario(scenario, planning_problem_set)

    scenario_file_mod = Path(__file__).parent.joinpath(f"./../scenarios/modified_scenario.xml")
    scenario_mod, planning_problem_mod = CommonRoadFileReader(scenario_file_mod).open()

    graph, step_start, step_end, planning_problem_mod, clcs = reach_flow.create_reach_graph(
        "scenarios/modified_scenario.xml", "Behind_V311"
    )
    reach_flow.draw_reach_sets_end(step_end, scenario_mod, planning_problem_mod, graph, clcs)

    area_modified = reach_flow.compute_drivable_area("scenarios/modified_scenario.xml")
    reach_flow.plot(area_original, area_modified)

    # target_vehicle = next((veh for veh in scenario.dynamic_obstacles if veh.obstacle_id == 30), None)
    # print(f"30pos after opt {target_vehicle.initial_state.position[0]}")


run_full_optimization_pipeline("scenarios/BEL_Aarschot-6_1_T-1.xml", [("311", "position")])
