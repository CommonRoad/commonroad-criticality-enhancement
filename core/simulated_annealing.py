import nevergrad as ng


def run_simulated_annealing(
    num_vars: int, lower_bound: float = 0, upper_bound: float = 100, budget: int = 10
):
    """
    Runs Simulated Annealing optimization using Nevergrad to maximize drivable area
    (or minimize any custom objective function).

    Parameters:
    - num_vars (int): The number of decision variables to optimize.
    - lower_bound (float): The minimum allowable value for decision variables (e.g., min velocity or position). Default is 0.
    - upper_bound (float): The maximum allowable value for decision variables (e.g., max velocity or position). Default is 100.
    - budget (int, optional): The number of optimization iterations (calls to the objective function). Default is 10.

    Returns:
    - best_vars (List[float]): The best decision variable values found during optimization.
    - best_area (float): The best drivable area found during optimization.
    """
    # Define the variable space
    parametrization = ng.p.Array(shape=(num_vars,)).set_bounds(lower_bound, upper_bound)

    # Choose Simulated Annealing optimizer
    optimizer = ng.optimization.optimizerlib.CMandAS2(
        parametrization=parametrization, budget=budget
    )

    # Optimization loop
    for _ in range(budget):
        candidate = optimizer.ask()
        loss = compute_drivable_area(candidate.args[0])
        optimizer.tell(candidate, loss)

    # Return best result
    best = optimizer.provide_recommendation()
    best_vars = best.value
    best_area = compute_drivable_area(best_vars)
    return best_vars, best_area


# TODO
# def optimize_with_method(method: str, ...) -> Tuple[List[float], float]:
#     if method == "Gradient":
#         return optimize_with_ecos(...)
#     elif method == "SA":
#         return simulated_annealing(...)
#     else:
#         raise ValueError("Unsupported optimization method.")

# TODO compare both approaches
