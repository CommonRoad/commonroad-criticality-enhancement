import sys
import time
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader

from src import bo, optimization, reach_flow, sa

# Get the root directory (two levels up from this file)
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.append(str(PROJECT_ROOT / "scenarios"))


def plot_area_over_time(original_area, gradient_area, sa_area, bo_area):
    """
    Plots drivable area over time steps for each method.

    Parameters:
    - original_area, gradient_area, sa_area, bo_area: Areas for each time step.
    """
    time_steps = list(range(len(original_area)))

    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, original_area, label="Original", color="gray", linestyle="--")
    plt.plot(time_steps, gradient_area, label="Gradient", color="blue")
    plt.plot(time_steps, sa_area, label="Simulated Annealing", color="green")
    plt.plot(time_steps, bo_area, label="Bayesian Optimization", color="orange")

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
    iterations: int = 30,
    a_ref_input: float = 1.0,
    sa_max_iter: int = 100,
    sa_initial_temp: float = 2000.0,
    budget: int = 50,
) -> None:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    print("Computing original drivable area...")
    area_original = reach_flow.compute_drivable_area(scenario_path)

    print("\nRunning Gradient-Based Optimization (ECOS)...")

    start = time.time()

    velocity_gradient, area_gradient = optimization.optimize(
        scenario,
        planning_problem_set,
        scenario_path,
        decision_variables=decision_variables,
        iterations=iterations,
        a_ref_input=a_ref_input,
    )
    end = time.time()

    print("\nRunning SA ...")
    start_sa = time.time()
    sa_best_params, sa_area = sa.run_sa_with_scipy(
        scenario_path=scenario_path,
        decision_variables=decision_variables,
        max_iter=sa_max_iter,
        initial_temp=sa_initial_temp,
    )
    end_sa = time.time()

    print("\nRunning Bayesian Optimization ...")
    start_bo = time.time()
    bo_best_params, bo_area = bo.run_bo_multi_variable(
        scenario_path=scenario_path,
        decision_variables=decision_variables,
        budget=budget,
    )
    end_bo = time.time()

    print(f"Sum of Original drivable area: {sum((area_original - a_ref_input) ** 2)}")
    print(f"Gradient optimization time: {end - start:.2f} seconds")
    print(f"Gradient optimization params (vel, x-pos, y-pos): {velocity_gradient}")
    print(f"Sum of Gradient drivable area: {sum((area_gradient - a_ref_input) ** 2)}")
    print(f"SA optimization time: {end_sa - start_sa:.2f} seconds")
    print(f"SA optimization params: {sa_best_params}")
    print(f"Sum of SA drivable area: {sum((sa_area - a_ref_input) ** 2)}")
    print(f"BO optimization time: {end_bo - start_bo:.2f} seconds")
    print(f"BO optimization params: {bo_best_params}")
    print(f"Sum of BO drivable area: {sum((bo_area - a_ref_input) ** 2)}")

    plot_area_over_time(area_original, area_gradient, sa_area, bo_area)


scenario_path1 = PROJECT_ROOT / "scenarios" / "DEU_Flensburg-94_1_T-1.xml"
scenario_path2 = PROJECT_ROOT / "scenarios" / "BEL_Aarschot-6_1_T-1.xml"
scenario_path3 = PROJECT_ROOT / "scenarios" / "ZAM_Over-1_1.xml"
scenario_path4 = PROJECT_ROOT / "scenarios" / "DEU_Moelln-7_1_T-1.xml"
scenario_path5 = PROJECT_ROOT / "scenarios" / "DEU_Lohmar-32_1_T-1.xml"
scenario_path6 = PROJECT_ROOT / "scenarios" / "ZAM_two_lanes_solid.xml"
scenario_path7 = PROJECT_ROOT / "scenarios" / "BEL_Putte-3_1_T-1.xml"
scenario_path8 = PROJECT_ROOT / "scenarios" / "C-DEU_B471-1_3_T-1.xml"
scenario_path9 = PROJECT_ROOT / "scenarios" / "DEU_IV21-2_1_T-1.xml"
scenario_path10 = PROJECT_ROOT / "scenarios" / "USA_US101-11_4_T-1.xml"


# Run each of those with the corresponding iteration counts from the table to reproduce it
run_comparison_pipeline(
    str(scenario_path1),
    decision_variables=[("ego", "velocity"), ("ego", "position")],
    iterations=50,
    a_ref_input=1.0,
    sa_max_iter=200,
    sa_initial_temp=500.0,
    budget=500,
)
# run_comparison_pipeline(
#     str(scenario_path2),
#     decision_variables=[("ego", "velocity"), ("ego", "position")],
#     iterations=30,
#     a_ref_input=1.0,
#     sa_max_iter=50,
#     sa_initial_temp=500.0,
#     budget=150,
# )
# run_comparison_pipeline(
#     str(scenario_path3),
#     decision_variables=[("ego", "velocity"), ("ego", "position")],
#     iterations=30,
#     a_ref_input=1.0,
#     sa_max_iter=50,
#     sa_initial_temp=500.0,
#     budget=150,
# )
# run_comparison_pipeline(
#     str(scenario_path4),
#     decision_variables=[("ego", "velocity"), ("ego", "position")],
#     iterations=30,
#     a_ref_input=1.0,
#     sa_max_iter=50,
#     sa_initial_temp=500.0,
#     budget=150,
# )
# run_comparison_pipeline(
#     str(scenario_path5),
#     decision_variables=[("ego", "velocity"), ("ego", "position")],
#     iterations=30,
#     a_ref_input=1.0,
#     sa_max_iter=50,
#     sa_initial_temp=500.0,
#     budget=150,
# )
# run_comparison_pipeline(
#     str(scenario_path6),
#     decision_variables=[("ego", "velocity"), ("ego", "position")],
#     iterations=30,
#     a_ref_input=1.0,
#     sa_max_iter=50,
#     sa_initial_temp=500.0,
#     budget=150,
# )
# run_comparison_pipeline(
#     str(scenario_path7),
#     decision_variables=[("ego", "velocity"), ("ego", "position")],
#     iterations=30,
#     a_ref_input=1.0,
#     sa_max_iter=50,
#     sa_initial_temp=500.0,
#     budget=150,
# )
# run_comparison_pipeline(
#     str(scenario_path8),
#     decision_variables=[("ego", "velocity"), ("ego", "position")],
#     iterations=28,
#     a_ref_input=1.0,
#     sa_max_iter=50,
#     sa_initial_temp=500.0,
#     budget=150,
# )
# run_comparison_pipeline(
#     str(scenario_path9),
#     decision_variables=[("ego", "velocity"), ("ego", "position")],
#     iterations=30,
#     a_ref_input=1.0,
#     sa_max_iter=50,
#     sa_initial_temp=500.0,
#     budget=150,
# )
# run_comparison_pipeline(
#     str(scenario_path10),
#     decision_variables=[("ego", "velocity"), ("ego", "position")],
#     iterations=30,
#     a_ref_input=1.0,
#     sa_max_iter=50,
#     sa_initial_temp=500.0,
#     budget=150,
# )
