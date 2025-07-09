from pathlib import Path
from typing import List, Tuple

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from scipy.optimize import dual_annealing

from file_modification import apply_variables_to_scenario
from reach_flow import compute_drivable_area


def objective_wrapper(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    decision_variables: List[Tuple[str, str]],
    a_ref: float = 1.0,
):
    """
    Creates an objective function for optimization that applies decision variables,
    runs the pipeline, and returns the total drivable area as a scalar.

    Parameters
    ----------
    scenario : Scenario
        The modified CommonRoad scenario.

    planning_problem_set : PlanningProblemSet
        The associated planning problem set.

    decision_variables : List[Tuple[str, str]]
        The (vehicle_id, variable_type) for each parameter.

    a_ref : float, optional
        The reference area. Defaults to 1.0.

    Returns
    -------
    Callable[[List[float]], float]
        Objective function that takes parameter values and returns drivable area.
    """

    def objective(params: List[float]) -> float:
        try:
            updated_path = apply_variables_to_scenario(scenario, planning_problem_set, list(params), decision_variables)
            area = compute_drivable_area(updated_path)
            total_squared_area = (sum(area) - a_ref) ** 2
            return total_squared_area
        except Exception as e:
            print(f"[Warning] Area for this value could not be computed. {e}")
            # Return a value for the area bigger than the other values, so this infeasible parameter will not be used for further sampling
            return 1e6

    return objective


def run_sa_with_scipy(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    max_iter: int = 500,
    initial_temp: float = 2000.0,
    a_ref: float = 1.0,
) -> Tuple[List[float], List[float]]:
    """
    Runs simulated annealing optimization on decision variables to minimize drivable area.

    Parameters
    ----------
    scenario_path : str
        Path to the CommonRoad scenario XML file.

    decision_variables : List[Tuple[str, str]]
        Variables to optimize, e.g. [("ego", "velocity")].

    max_iter : int, optional
        Maximum number of iterations for the optimizer. Defaults to 500.

    initial_temp : float, optional
        Initial temperature parameter for simulated annealing. Default is 2000.0.

    a_ref : float, optional
        The reference area. Defaults to 1.0.

    Returns
    -------
    best_params : List[float]
        Best parameter values found by the optimizer.

    best_area : List[float]
        Drivable area array computed with the best parameters.
    """

    # Load scenario
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # Compute bounds based on original state
    bounds = []
    expanded_decision_variables = []
    for vehicle_id, variable_type in decision_variables:
        if vehicle_id == "ego":
            vehicle = list(planning_problem_set.planning_problem_dict.values())[0]
        else:
            raise ValueError(f"Program supports only ego vehicle currently")

        # Get bounds based on variable type
        if variable_type == "velocity":
            v_original = vehicle.initial_state.velocity
            bounds.append((5, v_original + 30.0))
            expanded_decision_variables.append((vehicle_id, "velocity"))

        elif variable_type == "position":
            x_original, y_original = vehicle.initial_state.position
            expanded_decision_variables.append((vehicle_id, "x-position"))
            expanded_decision_variables.append((vehicle_id, "y-position"))
            bounds.append((x_original - 2.0, x_original + 2.0))
            bounds.append((y_original - 2.0, y_original + 2.0))

        elif variable_type == "x-position":
            x_original = vehicle.initial_state.position[0]
            expanded_decision_variables.append((vehicle_id, "x-position"))
            bounds.append((x_original - 2.0, x_original + 2.0))

        elif variable_type == "y-position":
            y_original = vehicle.initial_state.position[1]
            expanded_decision_variables.append((vehicle_id, "y-position"))
            bounds.append((y_original - 2.0, y_original + 2.0))

        else:
            raise ValueError(f"Unknown decision variable type: {variable_type}")

    # Prepare the objective function
    objective = objective_wrapper(scenario, planning_problem_set, expanded_decision_variables, a_ref)

    # Run SciPy's Simulated Annealing
    result = dual_annealing(objective, bounds=bounds, maxiter=max_iter, initial_temp=initial_temp)

    best_params = list(result.x)
    updated_path = apply_variables_to_scenario(scenario, planning_problem_set, best_params, expanded_decision_variables)
    best_area = compute_drivable_area(updated_path)

    return best_params, best_area
