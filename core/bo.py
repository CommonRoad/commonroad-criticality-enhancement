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
        return float("inf")


def run_bo_multi_variable(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    lower_bound: float,
    upper_bound: float,
    budget: int = 50,
    a_ref: float = 1.0,
) -> Tuple[List[float], List[float]]:
    """
    Runs Bayesian Optimization over multiple decision variables to minimize drivable area.

    Parameters:
    - scenario_path (str): The path to the CommonRoad scenario.
    - decision_variables (List[Tuple[str, str]]): Variables to optimize, e.g. [("ego", "velocity"), (31, "position")]
    - lower_bound (float): Min value each variable can take
    - upper_bound (float): Max value each variable can take
    - budget (int, optional): Number of evaluations allowed. Defaults to 50.
    - a_ref (float, optional): The reference area. Defaults to 1.0.

    Returns:
    - best_params (List[float]): Best parameter values found
    - best_area (List[float]): The minimized drivable area array
    """
    scenario_file = Path(__file__).parent.joinpath(f"./../{scenario_path}")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    dim = len(decision_variables)

    # Define search space with bounds for each decision variable
    space = [Real(lower_bound, upper_bound) for _ in range(dim)]

    def wrapped_objective(params):
        return objective_multi_var(scenario, planning_problem_set, params, decision_variables, a_ref)

    # Run Gaussian Process-based Bayesian Optimization
    result = gp_minimize(
        wrapped_objective,
        space,
        n_calls=budget,
        random_state=42,
        verbose=False,
    )

    best_params = result.x

    updated_scenario_path = apply_variables_to_scenario(scenario, planning_problem_set, best_params, decision_variables)
    best_area = compute_drivable_area(updated_scenario_path)
    return best_params, best_area
