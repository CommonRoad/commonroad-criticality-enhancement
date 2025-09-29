import logging
from typing import List, Tuple

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from numpy import ndarray
from skopt import gp_minimize
from skopt.space import Real

from .file_modification import apply_variables_to_scenario
from .reach_flow import compute_drivable_area

_LOGGER = logging.getLogger(__name__)


def objective_multi_var(
    scenario: Scenario,
    path: str,
    planning_problem_set: PlanningProblemSet,
    params: List[float],
    decision_variables: List[Tuple[str, str]],
    a_ref: float = 1.0,
) -> float:
    """
    Applies decision variables (velocity or position changes), runs the pipeline, and returns drivable area.

    Parameters
    ----------
    scenario : Scenario
        The CommonRoad scenario.

    path : str
        The path of the scenario.

    planning_problem_set : PlanningProblemSet
        The associated planning problem set.

    params : List[float]
        Values corresponding to the decision variables.

    decision_variables : List[Tuple[str, str]]
        The (vehicle_id, variable_type) for each param.

    a_ref : float, optional
        The reference area. Defaults to 1.0.

    Returns
    -------
    float
        The drivable area (we are minimizing it).
    """
    try:
        updated_scenario_path = apply_variables_to_scenario(
            scenario, path, planning_problem_set, params, decision_variables, sa=False
        )
        area = compute_drivable_area(updated_scenario_path)
        total_squared_area = (sum(area) - a_ref) ** 2
        return total_squared_area
    except Exception as e:
        _LOGGER.error(f"Area for this value could not be computed. {e} Returning 1e20 for this value.")
        # Return a value for the area bigger than the other values, so this infeasible parameter will not be used for further sampling
        return 1e20


def run_bo_multi_variable(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    budget: int = 100,
    a_ref: float = 1.0,
    callback: any = None,
) -> Tuple[List[float], ndarray]:
    """
    Runs Bayesian Optimization over multiple decision variables to minimize drivable area.

    Parameters
    ----------
    scenario_path : str
        The path to the CommonRoad scenario.

    decision_variables : List[Tuple[str, str]]
        Variables to optimize, e.g. [("ego", "velocity")]

    budget : int, optional
        Number of evaluations allowed. Defaults to 100.

    a_ref : float, optional
        The reference area. Defaults to 1.0.

    callback : any
        The callbacks are directly passed onto the underlying BO method.

    Returns
    -------
    best_params : List[float]
        Best parameter values found.

    best_area : ndarray
        The minimized drivable area array.
    """

    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # Compute bounds based on decision variables
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
            x = float(pos[0])
            y = float(pos[1])
            space.append(Real(x - 2, x + 2))  # x
            space.append(Real(y - 2, y + 2))  # y
            expanded_decision_variables.extend([(vehicle_id, "x-position"), (vehicle_id, "y-position")])

        elif variable_type == "x-position":
            x = float(vehicle.initial_state.position[0])
            space.append(Real(x - 2, x + 2))
            expanded_decision_variables.append((vehicle_id, "x-position"))

        elif variable_type == "y-position":
            y = float(vehicle.initial_state.position[1])
            space.append(Real(y - 2, y + 2))
            expanded_decision_variables.append((vehicle_id, "y-position"))

        else:
            raise ValueError(f"Unknown variable type: {variable_type}")

    # Prepare the objective function
    def wrapped_objective(params):
        return objective_multi_var(
            scenario, scenario_path, planning_problem_set, params, expanded_decision_variables, a_ref
        )

    # Run Gaussian Process-based Bayesian Optimization
    result = gp_minimize(wrapped_objective, space, n_calls=budget, random_state=42, verbose=False, callback=callback)

    # Apply the best parameters
    best_params = result.x
    updated_scenario_path = apply_variables_to_scenario(
        scenario, scenario_path, planning_problem_set, best_params, expanded_decision_variables, sa=False
    )
    best_area = compute_drivable_area(updated_scenario_path)
    return best_params, best_area
