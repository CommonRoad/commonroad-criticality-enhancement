import copy
import sys
import time
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_criticality_enhancement import optimization, reach_flow, sa

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.append(str(PROJECT_ROOT / "scenarios"))


def plot_area_over_time(original_area, gradient_area, sa_area):
    """
    Plots drivable area over time steps for each method.

    Parameters:
    - original_area, gradient_area, sa_area: Lists or arrays of float values for each time step.
    """
    time_steps = list(range(len(original_area)))  # assume same length for all

    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, original_area, label="Original", color="gray", linestyle="--")
    plt.plot(time_steps, gradient_area, label="Gradient", color="blue")
    plt.plot(time_steps, sa_area, label="Simulated Annealing", color="green")

    plt.xlabel("Time Step")
    plt.ylabel("Drivable Area")
    plt.title("Drivable Area Over Time")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.show()


def run_comparison_pipeline(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    iterations: int = 10,
    a_ref_input: float = 1.0,
    sa_max_iter: int = 500,
    sa_initial_temp: float = 2000.0,
) -> None:
    # Load scenario and compute the reachability graph and drivable area
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()
    graph, step_start, step_end, planning_problem, clcs = reach_flow.create_reach_graph(scenario, planning_problem_set)
    reach_flow.draw_reach_sets_end(step_end, scenario, planning_problem, graph, clcs)

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

    print("\nRunning SA Optimization...")
    start_sa = time.time()
    sa_best_params, sa_area = sa.run_sa_with_scipy(
        copy.deepcopy(scenario),
        copy.deepcopy(planning_problem_set),
        decision_variables=decision_variables,
        max_iter=sa_max_iter,
        initial_temp=sa_initial_temp,
    )
    end_sa = time.time()

    print(f"Sum of Original drivable area: {sum(area_original)}")
    print(f"Gradient optimization time: {end - start:.2f} seconds")
    print(f"Gradient optimization params (vel, x-pos, y-pos): {params_gradient}")
    print(f"Sum of Gradient drivable area: {sum(area_gradient)}")
    print(f"SA optimization time: {end_sa - start_sa:.2f} seconds")
    print(f"SA optimization velocity: {sa_best_params}")
    print(f"Sum of SA drivable area: {sum(sa_area)}")

    plot_area_over_time(area_original, area_gradient, sa_area)


scenario_path = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"

run_comparison_pipeline(
    scenario_path=str(scenario_path),
    decision_variables=[("ego", "velocity"), ("ego", "position")],
    iterations=10,
    a_ref_input=1.0,
    sa_max_iter=30,
    sa_initial_temp=500.0,
)
