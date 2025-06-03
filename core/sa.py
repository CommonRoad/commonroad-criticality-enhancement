from pathlib import Path
from typing import List, Tuple

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from file_modification import apply_variables_to_scenario
from reach_flow import compute_drivable_area
from scipy.optimize import dual_annealing


def objective_wrapper(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    decision_variables: List[Tuple[str, str]],
):
    """
    Creates an objective function for optimization that applies decision variables,
    runs the pipeline, and returns the total drivable area as a scalar.

    Parameters:
    - scenario (Scenario): The modified CommonRoad scenario.
    - planning_problem_set (PlanningProblemSet): The associated planning problem set.
    - decision_variables (List[Tuple[str, str]]): The (vehicle_id, variable_type) for each parameter.

    Returns:
    - Callable[[List[float]], float]: Objective function that takes parameter values and returns drivable area.
    """

    def objective(params: List[float]) -> float:
        try:
            updated_path = apply_variables_to_scenario(
                scenario, planning_problem_set, list(params), decision_variables
            )
            area = compute_drivable_area(updated_path)
            return sum(area)
        except Exception as e:
            print(f"[Error] {e}")
            return float("inf")

    return objective


def run_sa_with_scipy(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    lower_bound: float,
    upper_bound: float,
    max_iter: int = 500,
    initial_temp: float = 5230.0,
) -> Tuple[List[float], List[float]]:
    """
    Runs simulated annealing optimization on decision variables to minimize drivable area.

    Parameters:
    - scenario_path (str): Path to the CommonRoad scenario XML file.
    - decision_variables (List[Tuple[str, str]]): Variables to optimize, e.g. [("ego", "velocity")].
    - lower_bound (float): Minimum value each decision variable can take.
    - upper_bound (float): Maximum value each decision variable can take.
    - max_iter (int, optional): Maximum number of iterations for the optimizer.
    - initial_temp (float, optional): Initial temperature parameter for simulated annealing.

    Returns:
    - best_params (List[float]): Best parameter values found by the optimizer.
    - best_area (List[float]): Drivable area array computed with the best parameters.
    """
    # Load scenario
    scenario_file = Path(__file__).parent.joinpath(f"./../{scenario_path}")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    # Prepare the objective function
    objective = objective_wrapper(scenario, planning_problem_set, decision_variables)

    # Define bounds for each variable
    dim = len(decision_variables)
    bounds = [(lower_bound, upper_bound)] * dim

    # Run SciPy's Simulated Annealing
    result = dual_annealing(objective, bounds=bounds, maxiter=max_iter, initial_temp=initial_temp)

    best_params = list(result.x)
    updated_path = apply_variables_to_scenario(
        scenario, planning_problem_set, best_params, decision_variables
    )
    best_area = compute_drivable_area(updated_path)

    return best_params, best_area
