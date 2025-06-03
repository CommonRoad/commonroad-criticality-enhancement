import os
import sys
from pathlib import Path
from typing import List, Tuple

# Add your project structure to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "core")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenario")))

import matplotlib.pyplot as plt
import reach_flow
from CMandAS2 import run_sa_multi_variable
from commonroad.common.file_reader import CommonRoadFileReader
from optimization import optimize


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
    sa_bounds: Tuple[float, float] = (5.0, 25.0),
    sa_budget: int = 10,
) -> None:
    full_path = Path(__file__).parent.joinpath(f"./../{scenario_path}")
    scenario, planning_problem_set = CommonRoadFileReader(full_path).open()

    print("Computing original drivable area...")
    area_original = reach_flow.compute_drivable_area(scenario_path)

    print("\nRunning Gradient-Based Optimization (ECOS)...")
    velocity_gradient = optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
    )
    area_gradient = reach_flow.compute_drivable_area("scenarios/modified_scenario.xml")

    print("\nRunning Simulated Annealing (SA)...")
    sa_best_params, sa_area = run_sa_multi_variable(
        scenario_path=scenario_path,
        decision_variables=decision_variables,
        lower_bound=sa_bounds[0],
        upper_bound=sa_bounds[1],
        budget=sa_budget,
    )

    plot_area_over_time(area_original, area_gradient, sa_area)


if __name__ == "__main__":
    run_comparison_pipeline(
        "scenarios/DEU_Reutlingen-5_1_T-1.xml",
        decision_variables=[("ego", "velocity")],
        iterations=10,
        a_ref_input=1.0,
        sa_bounds=(0.0, 20.0),
        sa_budget=10,
    )
