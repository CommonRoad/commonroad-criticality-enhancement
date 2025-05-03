import os

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import LongitudinalState, PMState
from commonroad.scenario.trajectory import Trajectory
from cvxpy import Constraint, Minimize, Objective, Problem, Variable, quad_form
from vehiclemodels import parameters_vehicle3


def regenerate_ego_prediction(planning_problem, steps=30, delta_t=0.1):
    orientation = planning_problem.initial_state.orientation
    velocity = planning_problem.initial_state.velocity
    x, y = planning_problem.initial_state.position
    t = planning_problem.initial_state.time_step

    state_list = []
    for i in range(steps):
        t += 1
        x += velocity * delta_t * np.cos(orientation)
        y += velocity * delta_t * np.sin(orientation)
        longitudinal_pos = velocity * delta_t * (i + 1)

        state = PMState(
            time_step=t,
            position=np.array([longitudinal_pos, 0.0]),
            velocity=velocity * np.cos(orientation),
            velocity_y=velocity * np.sin(orientation),
        )
        state_list.append(state)

    trajectory = Trajectory(initial_time_step=1, state_list=state_list)

    # Define vehicle shape
    vehicle3 = parameters_vehicle3.parameters_vehicle3()
    shape = Rectangle(length=vehicle3.l, width=vehicle3.w)

    # Return new prediction and dynamic obstacle
    prediction = TrajectoryPrediction(trajectory=trajectory, shape=shape)
    obstacle = DynamicObstacle(
        obstacle_id=100,
        obstacle_type=ObstacleType.CAR,
        obstacle_shape=shape,
        initial_state=planning_problem.initial_state,
        prediction=prediction,
    )

    return obstacle


def save_modified_scenario(scenario, planning_problem_set) -> str:
    # Regenerate ego prediction with updated velocity and trajectory
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
    updated_ego_vehicle = regenerate_ego_prediction(planning_problem)

    # Remove the old ego vehicle
    scenario.dynamic_obstacles[:] = [
        obs for obs in scenario.dynamic_obstacles if obs.obstacle_id != 100
    ]

    # Add the updated ego vehicle
    scenario.dynamic_obstacles.append(updated_ego_vehicle)

    temp_file = os.path.join("scenarios", "modified_scenario.xml")
    writer = CommonRoadFileWriter(scenario, planning_problem_set)
    writer.write_to_file(temp_file, overwrite_existing_file=OverwriteExistingFile.ALWAYS)
    print("The new scenario was saved in modified_scenario.xml")
    return "modified_scenario"
