from typing import List, Tuple

import nevergrad as ng
from file_modification import apply_variables_to_scenario
from reach_flow import compute_drivable_area


def objective_multi_var(params: List[float], decision_variables: List[Tuple[str, str]]) -> float:
    """
    Applies decision variables (velocity or position changes), runs the pipeline, and returns drivable area.

    Parameters:
    - params (List[float]): Values corresponding to the decision_variables
    - decision_variables (List[Tuple[str, str]]): The (vehicle_id, variable_type) for each param

    Returns:
    - float: The drivable area (we are minimizing it)
    """
    try:
        # Apply the changes to the scenario/planning problem
        updated_scenario_path = apply_variables_to_scenario(params, decision_variables)

        # Compute drivable area with your pipeline
        area = compute_drivable_area(updated_scenario_path)
        return area  # because we want to minimize
    except Exception as e:
        print(f"Error during simulation: {e}")
        return float("inf")


def run_sa_multi_variable(
    decision_variables: List[Tuple[str, str]],
    lower_bound: float,
    upper_bound: float,
    budget: int = 100,
) -> Tuple[List[float], float]:
    """
    Runs SA optimization over multiple decision variables to minimize drivable area.

    Parameters:
    - decision_variables (List[Tuple[str, str]]): Variables to optimize, e.g. [("ego", "velocity"), (31, "position")]
    - lower_bound (float): Min value each variable can take
    - upper_bound (float): Max value each variable can take
    - budget (int): Number of evaluations allowed

    Returns:
    - best_params (List[float]): Best parameter values found
    - best_area (float): The minimized summed up drivable area
    """
    dim = len(decision_variables)
    parametrization = ng.p.Array(shape=(dim,)).set_bounds(lower_bound, upper_bound)
    optimizer = ng.optimization.optimizerlib.CMandAS2(
        parametrization=parametrization, budget=budget
    )

    for _ in range(budget):
        candidate = optimizer.ask()
        loss = objective_multi_var(candidate.args[0], decision_variables)
        optimizer.tell(candidate, loss)

    best = optimizer.provide_recommendation()
    best_params = best.value
    best_area = objective_multi_var(best_params, decision_variables)

    return best_params, best_area


# TODO
# def optimize_with_method(method: str, ...) -> Tuple[List[float], float]:
#     if method == "Gradient":
#         return optimize_with_ecos(...)
#     elif method == "SA":
#         return simulated_annealing(...)
#     else:
#         raise ValueError("Unsupported optimization method.")

# TODO compare both approaches
