import sys
import time
from pathlib import Path

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_criticality_enhancement import optimization, reach_flow

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.append(str(PROJECT_ROOT / "scenarios"))


def run_full_optimization_pipeline(
    scenario_path: str,
    decision_variables: list,
    iterations: int = 10,
    a_ref_input: float = 1.0,
    semantics: str = "true",
) -> None:
    # Load scenario and compute the reachability graph and drivable area
    print("starting full optimization")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # Get ego vehicle's initial position
    ego_state = list(planning_problem_set.planning_problem_dict.values())[0].initial_state
    ego_position = np.array([ego_state.position[0], ego_state.position[1]])

    # Find the lanelet that contains this position
    ego_lanelet = None
    for lanelet in scenario.lanelet_network.lanelets:
        if lanelet.polygon.contains_point(ego_position):
            ego_lanelet = lanelet
            break

    if ego_lanelet is None:
        print("Ego not in any lanelet")
    else:
        # Traffic signs IDs attached to lanelet
        sign_ids = ego_lanelet.traffic_signs

        # Now get traffic sign objects from scenario (assuming scenario has these)
        for sign_id in sign_ids:
            traffic_sign = scenario.traffic_signs[sign_id]

            # Example: check if the traffic sign is a speed limit
            if traffic_sign.type == "SpeedLimit":
                speed_limit = traffic_sign.value  # or some attribute holding speed limit
                print(f"Speed limit on lanelet: {speed_limit} km/h")

    print(f"Ego not in lanelet{ego_lanelet}")

    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        scenario, planning_problem_set, semantics
    )
    print(f"initial velocity: {planning_problem.initial_state.velocity}")
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)
    area_original = reach_flow.compute_drivable_area(scenario, planning_problem_set, semantics)

    # Run gradient-based optimization
    start = time.time()
    final_params, area_modified = optimization.optimize(
        scenario,
        planning_problem_set,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
        semantics=semantics,
    )
    end = time.time()
    print("final_params:", final_params)

    # Create reach graph for modified scenario
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
        scenario, planning_problem_set, semantics
    )

    # Draw reachable sets for modified scenario
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    # Plot areas
    reach_flow.plot(area_original, area_modified)
    print(f"original sum: {sum(area_original)}")
    print(f"modified sum: {sum(area_modified)}")
    print(f"Gradient optimization time: {end - start:.2f} seconds")


scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"
run_full_optimization_pipeline(str(scenario_path), [("ego", "velocity")])
