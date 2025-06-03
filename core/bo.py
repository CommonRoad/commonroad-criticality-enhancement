from pathlib import Path
from typing import List, Tuple

import nevergrad as ng
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from file_modification import apply_variables_to_scenario
from reach_flow import compute_drivable_area


def objective_multi_var(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    params: List[float],
    decision_variables: List[Tuple[str, str]],
) -> Tuple[float, List[float]]:
    """
    Applies decision variables (velocity or position changes), runs the pipeline, and returns drivable area.

    Parameters:
    - scenario (Scenario): The modified CommonRoad scenario.
    - planning_problem_set (PlanningProblemSet): The associated planning problem set.
    - params (List[float]): Values corresponding to the decision_variables
    - decision_variables (List[Tuple[str, str]]): The (vehicle_id, variable_type) for each param

    Returns:
    - float: The drivable area (we are minimizing it)
    - area (List[float]): The drivable area
    """
    try:
        updated_scenario_path = apply_variables_to_scenario(
            scenario, planning_problem_set, params, decision_variables
        )
        area = compute_drivable_area(updated_scenario_path)
        return sum(area), area
    except Exception as e:
        print(f"Error during simulation: {e}")
        return float("inf"), []


def run_bo_multi_variable(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    lower_bound: float,
    upper_bound: float,
    budget: int = 50,
) -> Tuple[List[float], List[float]]:
    """
    Runs Bayesian Optimization over multiple decision variables to minimize drivable area.

    Parameters:
    - scenario_path (str): The path to the CommonRoad scenario.
    - decision_variables (List[Tuple[str, str]]): Variables to optimize, e.g. [("ego", "velocity"), (31, "position")]
    - lower_bound (float): Min value each variable can take
    - upper_bound (float): Max value each variable can take
    - budget (int, optional): Number of evaluations allowed. Defaults to 50.

    Returns:
    - best_params (List[float]): Best parameter values found
    - best_area (List[float]): The minimized drivable area array
    """
    scenario_file = Path(__file__).parent.joinpath(f"./../{scenario_path}")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    dim = len(decision_variables)
    parametrization = ng.p.Array(shape=(dim,)).set_bounds(lower_bound, upper_bound)

    # Use Nevergrad's Bayesian Optimization optimizer
    optimizer = ng.optimizers.BayesianOptimization(parametrization=parametrization, budget=budget)

    for _ in range(budget):
        candidate = optimizer.ask()
        # candidate.value is a numpy array - convert to list
        loss, _ = objective_multi_var(
            scenario, planning_problem_set, candidate.value.tolist(), decision_variables
        )
        optimizer.tell(candidate, loss)

    best = optimizer.provide_recommendation()
    best_params = best.value.tolist()
    _, best_area = objective_multi_var(
        scenario, planning_problem_set, best_params, decision_variables
    )

    return best_params, best_area
