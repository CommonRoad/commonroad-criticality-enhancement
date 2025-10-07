import copy
import sys
import time
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_criticality_enhancement import bo, optimization, reach_flow

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.append(str(PROJECT_ROOT / "scenarios"))


def plot_area_over_time(original_area, gradient_area, bo_area):
    """
    Plots drivable area over time steps for each method.

    Parameters:
    - original_area, gradient_area, bo_area: Lists or arrays of float values for each time step.
    """
    time_steps = list(range(len(original_area)))  # assume same length for all

    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, original_area, label="Original", color="gray", linestyle="--")
    plt.plot(time_steps, gradient_area, label="Gradient", color="blue")
    plt.plot(time_steps, bo_area, label="Bayesian Opt", color="red")

    plt.xlabel("Time Step")
    plt.ylabel("Drivable Area")
    plt.title("Drivable Area Over Time")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.margins(x=0)
    plt.tight_layout()
    plt.show()


def run_comparison_pipeline(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    iterations: int = 10,
    a_ref_input: float = 1.0,
    budget: int = 10,
) -> None:
    # Load scenario and compute the reachability graph and drivable area
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    print("Computing original drivable area...")
    area_original = reach_flow.compute_drivable_area(scenario, planning_problem_set)

    print("\nRunning Gradient-Based Optimization (ECOS)...")

    start = time.time()

    params_gradient, area_gradient = optimization.optimize(
        copy.deepcopy(scenario),
        copy.deepcopy(planning_problem_set),
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
    )
    end = time.time()

    print("\nRunning Bayesian Optimization ...")
    start_bo = time.time()
    bo_best_params, bo_area = bo.run_bo_multi_variable(
        copy.deepcopy(scenario),
        copy.deepcopy(planning_problem_set),
        decision_variables=decision_variables,
        budget=budget,
    )
    end_bo = time.time()

    print(f"Sum of Original drivable area: {sum(area_original)}")
    print(f"Gradient optimization time: {end - start:.2f} seconds")
    print(f"Gradient optimization params (vel, x-pos, y-pos): {params_gradient}")
    print(f"Sum of Gradient drivable area: {sum(area_gradient)}")
    print(f"BO optimization time: {end_bo - start_bo:.2f} seconds")
    print(f"BO optimization params: {bo_best_params}")
    print(f"Sum of BO drivable area: {sum(bo_area)}")

    plot_area_over_time(area_original, area_gradient, bo_area)


scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"

run_comparison_pipeline(
    str(scenario_path),
    decision_variables=[("ego", "position"), ("ego", "velocity")],
    iterations=1,
    a_ref_input=1.0,
    budget=500,
)
