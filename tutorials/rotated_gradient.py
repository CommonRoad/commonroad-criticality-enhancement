import sys
from pathlib import Path

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import Trajectory
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import KSState
from matplotlib import pyplot as plt

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
# Add src and scenario directories to sys.path
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(PROJECT_ROOT / "scenarios"))
import reach_flow
from optimization import optimize


def rotate_point_90_ccw(point):
    # Rotate the point by 90 degrees
    x, y = point
    return np.array([-y, x])


def rotate_goal_region(goal_region, origin=(0, 0)):
    # Rotate the goal region by 90 degrees
    for goal_state in goal_region.state_list:
        if goal_state.position is not None:
            pos = goal_state.position
            pos.center = rotate_polyline_90_ccw(pos.center)
            pos.orientation += np.pi / 2


def rotate_polyline_90_ccw(polyline: np.ndarray) -> np.ndarray:
    # Rotates a polyline by 90 degrees
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
        obs.initial_state.orientation += np.pi / 2
        rotate_dynamic_obstacle_trajectory(obs)

    # Rotate initial state of ego vehicle (PlanningProblem)
    for pp in planning_problem_set.planning_problem_dict.values():
        # Not every scenario has a goal region, so uncomment accordingly
        # rotate_goal_region(pp.goal)
        pos = np.asarray(pp.initial_state.position)
        rotated_pos = rotate_point_90_ccw(pos)
        pp.initial_state.position = rotated_pos
        pp.initial_state.orientation += np.pi / 2

    return scenario, planning_problem_set


def compare_plot(
    area_original_rotated: np.ndarray, area_original: np.ndarray, area_rotated: np.ndarray, area_modified: np.ndarray
) -> None:
    """
    Plots a comparison of drivable area over time for original, rotated and modified scenarios.

    Parameters:
    - area_original (np.ndarray): 1D array of drivable area values for the original scenario.
    - area_modified (np.ndarray): 1D array of drivable area values for the modified scenario.

    Returns:
    - None
    """
    time_steps = np.arange(len(area_original))
    plt.figure(figsize=(10, 5))
    plt.plot(time_steps, area_original, label="Original", color="gray", linestyle="--")
    plt.plot(time_steps, area_original_rotated, label="Original Rotated", color="blue")
    plt.plot(time_steps, area_rotated, label="Modified Rotated", color="red")
    plt.plot(time_steps, area_modified, label="Modified", color="green")
    plt.xlabel("Time Step")
    plt.ylabel("Drivable Area")
    plt.title("Comparison of Drivable Areas Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def run_full_optimization_pipeline(
    rotated_scenario_path, scenario_path: str, decision_variables: list, iterations: int = 5, a_ref_input: float = 1.0
) -> None:
    # Create reach graph for rotated original scenario and compute rotated original area
    scenario, planning_problem_set = CommonRoadFileReader(rotated_scenario_path).open()
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(rotated_scenario_path)
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    area_original_rotated = reach_flow.compute_drivable_area(rotated_scenario_path)

    # Optimize rotated scenario
    final_velocity_rot, area_rotated = optimize(
        scenario,
        planning_problem_set,
        rotated_scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
    )

    # Create reach graph for original scenario and compute original area
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(scenario_path)
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

    area_original = reach_flow.compute_drivable_area(scenario_path)

    # Optimize original scenario
    final_velocity, area_modified = optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
    )
    print("final_params_rotated:", final_velocity_rot)
    print("final_params:", final_velocity)

    compare_plot(area_original_rotated, area_original, area_rotated, area_modified)


scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Guetersloh-65_2_T-1.xml"
# scenario_path = PROJECT_ROOT / "scenarios" / "USA_US101-8_1_T-1.xml"
reader = CommonRoadFileReader(str(scenario_path))
scenario, pps = reader.open()

# Rotate scenario by 90 degrees
rotated_scenario, rotated_pps = rotate_scenario_90_ccw(scenario, pps)

# Save rotated scenario
rotated_path = PROJECT_ROOT / "scenarios" / "rotated_scenario.xml"
writer = CommonRoadFileWriter(rotated_scenario, rotated_pps, scenario.author, scenario.affiliation)
writer.write_to_file(rotated_path, OverwriteExistingFile.ALWAYS)

run_full_optimization_pipeline(rotated_path, scenario_path, [("ego", "velocity")])
