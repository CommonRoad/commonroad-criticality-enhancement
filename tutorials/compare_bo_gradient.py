import os
import sys
from pathlib import Path
from typing import List, Tuple

# Add your project structure to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "core")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenario")))

import time

import matplotlib.pyplot as plt
import reach_flow
from bo import run_bo_multi_variable
from commonroad.common.file_reader import CommonRoadFileReader
from optimization import optimize


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
    plt.tight_layout()
    plt.show()


def run_comparison_pipeline(
    scenario_path: str,
    decision_variables: List[Tuple[str, str]],
    iterations: int = 10,
    a_ref_input: float = 1.0,
    budget: int = 500,
) -> None:
    full_path = Path(__file__).parent.joinpath(f"./../{scenario_path}")
    scenario, planning_problem_set = CommonRoadFileReader(full_path).open()

    print("Computing original drivable area...")
    area_original = reach_flow.compute_drivable_area(scenario_path)

    print("\nRunning Gradient-Based Optimization (ECOS)...")

    start = time.time()

    velocity_gradient = optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
    )
    area_gradient = reach_flow.compute_drivable_area("scenarios/modified_scenario.xml")
    end = time.time()

    print("\nRunning Bayesian Optimization ...")
    start_bo = time.time()
    bo_best_params, bo_area = run_bo_multi_variable(
        scenario_path=scenario_path,
        decision_variables=decision_variables,
        budget=budget,
    )
    end_bo = time.time()

    print(f"Sum of Original drivable area: {sum(area_original)}")
    print(f"Gradient optimization time: {end - start:.2f} seconds")
    print(f"Gradient optimization velocity: {velocity_gradient} m/s")
    print(f"Sum of Gradient drivable area: {sum(area_gradient)}")
    print(f"BO optimization time: {end_bo - start_bo:.2f} seconds")
    print(f"BO optimization velocity: {bo_best_params} m/s")
    print(f"Sum of BO drivable area: {sum(bo_area)}")

    plot_area_over_time(area_original, area_gradient, bo_area)


if __name__ == "__main__":
    run_comparison_pipeline(
        # "scenarios/DEU_Reutlingen-5_1_T-1.xml",
        "scenarios/DEU_Flensburg-94_1_T-1.xml",
        decision_variables=[("ego", "velocity")],
        iterations=10,
        a_ref_input=1.0,
        budget=10,
    )
