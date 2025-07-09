from pathlib import Path
from typing import List, Tuple

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from file_modification import apply_variables_to_scenario
from reach_flow import compute_drivable_area
from skopt import gp_minimize
from skopt.space import Real


def objective_multi_var(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    params: List[float],
    decision_variables: List[Tuple[str, str]],
    a_ref: float = 1.0,
) -> float:
    """
    Applies decision variables (velocity or position changes), runs the pipeline, and returns drivable area.

    Parameters:
    - scenario (Scenario): The modified CommonRoad scenario.
    - planning_problem_set (PlanningProblemSet): The associated planning problem set.
    - params (List[float]): Values corresponding to the decision_variables
    - decision_variables (List[Tuple[str, str]]): The (vehicle_id, variable_type) for each param
    - a_ref (float, optional): The reference area. Defaults to 1.0.

    Returns:
    - float: The drivable area (we are minimizing it)
    """
    try:
        updated_scenario_path = apply_variables_to_scenario(scenario, planning_problem_set, params, decision_variables)
        area = compute_drivable_area(updated_scenario_path)
        total_squared_area = (sum(area) - a_ref) ** 2
        return total_squared_area
    except Exception as e:
        print(f"Error during simulation: {e}")
        # Return a value for the area bigger than the other values, so this infeasible parameter will not be used for further sampling
        return 1e6


def run_bo_multi_variable(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    budget: int = 50,
    a_ref: float = 1.0,
) -> Tuple[List[float], List[float]]:
    """
    Runs Bayesian Optimization over multiple decision variables to minimize drivable area.

    Parameters:
    - scenario_path (str): The path to the CommonRoad scenario.
    - decision_variables (List[Tuple[str, str]]): Variables to optimize, e.g. [("ego", "velocity"), (31, "position")]
    - budget (int, optional): Number of evaluations allowed. Defaults to 50.
    - a_ref (float, optional): The reference area. Defaults to 1.0.

    Returns:
    - best_params (List[float]): Best parameter values found
    - best_area (List[float]): The minimized drivable area array
    """
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # Compute bounds based on original state
    space = []
    expanded_decision_variables = []
    for vehicle_id, variable_type in decision_variables:
        if vehicle_id == "ego":
            vehicle = list(planning_problem_set.planning_problem_dict.values())[0]
        else:
            raise ValueError(f"Program supports only ego vehicle currently")

        # Set bounds
        if variable_type == "velocity":
            v_original = vehicle.initial_state.velocity
            space.append(Real(5, v_original + 30.0))
            expanded_decision_variables.append((vehicle_id, "velocity"))

        elif variable_type == "position":
            pos = vehicle.initial_state.position
            space.append(Real(pos[0] - 2, pos[0] + 2))  # x
            space.append(Real(pos[1] - 2, pos[1] + 2))  # y
            expanded_decision_variables.extend([(vehicle_id, "x-position"), (vehicle_id, "y-position")])

        elif variable_type == "x-position":
            x = vehicle.initial_state.position[0]
            space.append(Real(x - 2, x + 2))
            expanded_decision_variables.append((vehicle_id, "x-position"))

        elif variable_type == "y-position":
            y = vehicle.initial_state.position[1]
            space.append(Real(y - 2, y + 2))
            expanded_decision_variables.append((vehicle_id, "y-position"))

        else:
            raise ValueError(f"Unknown variable type: {variable_type}")

    def wrapped_objective(params):
        return objective_multi_var(scenario, planning_problem_set, params, expanded_decision_variables, a_ref)

    # Run Gaussian Process-based Bayesian Optimization
    result = gp_minimize(
        wrapped_objective,
        space,
        n_calls=budget,
        random_state=42,
        verbose=False,
    )

    best_params = result.x

    updated_scenario_path = apply_variables_to_scenario(
        scenario, planning_problem_set, best_params, expanded_decision_variables
    )
    best_area = compute_drivable_area(updated_scenario_path)
    return best_params, best_area
