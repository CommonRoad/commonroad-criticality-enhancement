import os
import sys

import reach_flow

# Add the parent directory (my_project) to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "core")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenario")))

from pathlib import Path

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import Trajectory
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import KSState
from optimization import optimize


def rotate_point_90_ccw(point):
    x, y = point
    return (-y, x)


def rotate_goal_region(goal_region, origin=(0, 0)):
    for goal_state in goal_region.state_list:
        if goal_state.position is not None:
            pos = goal_state.position
            pos.center = rotate_polyline_90_ccw(pos.center)
            pos.orientation += np.pi / 2


def rotate_polyline_90_ccw(polyline: np.ndarray) -> np.ndarray:
    # Ensure polyline is a (N, 2) NumPy array
    rotation_matrix = np.array([[0, -1], [1, 0]])
    return np.dot(polyline, rotation_matrix.T)


def rotate_dynamic_obstacle_trajectory(obstacle):
    if obstacle.prediction.trajectory is None:
        return

    original_traj = obstacle.prediction.trajectory
    rotated_states = []

    for state in original_traj.state_list:
        x, y = state.position
        x_rot, y_rot = rotate_point_90_ccw((x, y))

        # Copy all other fields, rotate only position
        rotated_state = KSState(
            time_step=state.time_step,
            position=np.array([x_rot, y_rot]),
            velocity=state.velocity,
            orientation=(state.orientation + np.pi / 2) % (2 * np.pi),
        )
        rotated_states.append(rotated_state)

    # Replace the trajectory with rotated one
    obstacle.prediction.trajectory = Trajectory(
        initial_time_step=original_traj.initial_time_step, state_list=rotated_states
    )


def rotate_scenario_90_ccw(scenario: Scenario, planning_problem_set: PlanningProblemSet):
    # Rotate lanelets
    for lanelet in scenario.lanelet_network.lanelets:
        lanelet.center_vertices = rotate_polyline_90_ccw(lanelet.center_vertices)
        lanelet.left_vertices = rotate_polyline_90_ccw(lanelet.left_vertices)
        lanelet.right_vertices = rotate_polyline_90_ccw(lanelet.right_vertices)

    # Rotate initial states of dynamic obstacles
    for obs in scenario.dynamic_obstacles:
        obs.initial_state.position = rotate_point_90_ccw(obs.initial_state.position)
        obs.initial_state.orientation = (obs.initial_state.orientation + np.pi / 2) % (2 * np.pi)

        rotate_dynamic_obstacle_trajectory(obs)

    # Rotate initial state of ego vehicle (PlanningProblem)
    for pp in planning_problem_set.planning_problem_dict.values():
        rotate_goal_region(pp.goal)
        pos = np.asarray(pp.initial_state.position)
        rotated_pos = rotate_point_90_ccw(pos)
        pp.initial_state.position = rotated_pos
        pp.initial_state.orientation += np.pi / 2

    return scenario, planning_problem_set


def run_full_optimization_pipeline(
    scenario_path: str, decision_variables: list, iterations: int = 1, a_ref_input: float = 1.0
) -> None:
    scenario_file = Path(__file__).parent.joinpath(f"./../{scenario_path}")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(scenario_path)
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    area_original = reach_flow.compute_drivable_area(scenario_path)
    #
    # final_velocity = optimize(
    #     scenario,
    #     planning_problem_set,
    #     scenario_path,
    #     decision_variables=decision_variables,
    #     iterations=iterations,
    #     a_ref_input=a_ref_input,
    # )
    # print("final_velocity:", final_velocity)
    #
    # scenario_file = Path(__file__).parent.joinpath(f"./../{scenario_path}")
    # scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
    #
    # graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(
    #     "scenarios/modified_scenario.xml"
    # )
    # reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)
    #
    # area_modified = reach_flow.compute_drivable_area("scenarios/modified_scenario.xml")
    # reach_flow.plot(area_original, area_modified)


scenario_path = "scenarios/USA_US101-8_1_T-1.xml"
reader = CommonRoadFileReader(scenario_path)
scenario, pps = reader.open()

rotated_scenario, rotated_pps = rotate_scenario_90_ccw(scenario, pps)

# Save rotated scenario
output_path = Path("scenarios/rotated_scenario.xml")
writer = CommonRoadFileWriter(rotated_scenario, rotated_pps, scenario.author, scenario.affiliation)
writer.write_to_file(output_path, OverwriteExistingFile.ALWAYS)
run_full_optimization_pipeline("scenarios/rotated_scenario.xml", [("ego", "velocity")])
